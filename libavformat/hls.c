/*
 * Apple HTTP Live Streaming demuxer
 * Copyright (c) 2010 Martin Storsjo
 * Copyright (c) 2013 Anssi Hannula
 * Copyright (c) 2021 Nachiket Tarate
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Apple HTTP Live Streaming demuxer
 * https://www.rfc-editor.org/rfc/rfc8216.txt
 */

#include "config_components.h"

#include "libavformat/http.h"
#include "libavutil/aes.h"
#include "libavutil/avstring.h"
#include "libavutil/avassert.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/dict.h"
#include "libavutil/time.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "avio_internal.h"
#include "id3v2.h"
#include "url.h"

#include "hls_sample_encryption.h"

#define INITIAL_BUFFER_SIZE 32768

#define MAX_FIELD_LEN 64
#define MAX_CHARACTERISTICS_LEN 512

#define MPEG_TIME_BASE 90000
#define MPEG_TIME_BASE_Q (AVRational){1, MPEG_TIME_BASE}

/*
 * An apple http stream consists of a playlist with media segment files,
 * played sequentially. There may be several playlists with the same
 * video content, in different bandwidth variants, that are played in
 * parallel (preferably only one bandwidth variant at a time). In this case,
 * the user supplied the url to a main playlist that only lists the variant
 * playlists.
 *
 * If the main playlist doesn't point at any variants, we still create
 * one anonymous toplevel variant for this, to maintain the structure.
 */

enum KeyType {
    KEY_NONE,
    KEY_AES_128,
    KEY_SAMPLE_AES
};

struct segment {
    int64_t duration;
    int64_t url_offset;
    int64_t size;
    char *url;
    char *key;
    enum KeyType key_type;
    uint8_t iv[16];
    /* associated Media Initialization Section, treated as a segment */
    struct segment *init_section;
};

struct rendition;

enum PlaylistType {
    PLS_TYPE_UNSPECIFIED,
    PLS_TYPE_EVENT,
    PLS_TYPE_VOD
};

/*
 * Each playlist has its own demuxer. If it currently is active,
 * it has an open AVIOContext too, and potentially an AVPacket
 * containing the next packet from this stream.
 */
struct playlist {
    char url[MAX_URL_SIZE];
    FFIOContext pb;
    uint8_t* read_buffer;
    AVIOContext *input;
    int input_read_done;
    AVIOContext *input_next;
    int input_next_requested;
    AVFormatContext *parent;
    int index;
    AVFormatContext *ctx;
    AVPacket *pkt;
    int has_noheader_flag;

    /* main demuxer streams associated with this playlist
     * indexed by the subdemuxer stream indexes */
    AVStream **main_streams;
    int n_main_streams;

    int finished;
    enum PlaylistType type;
    int64_t target_duration;
    int64_t start_seq_no;
    int time_offset_flag;
    int64_t start_time_offset;
    int n_segments;
    struct segment **segments;
    int needed;
    int broken;
    int64_t cur_seq_no;
    int64_t last_seq_no;
    int m3u8_hold_counters;
    int64_t cur_seg_offset;
    int64_t last_load_time;

    /* Currently active Media Initialization Section */
    struct segment *cur_init_section;
    uint8_t *init_sec_buf;
    unsigned int init_sec_buf_size;
    unsigned int init_sec_data_len;
    unsigned int init_sec_buf_read_offset;

    char key_url[MAX_URL_SIZE];
    uint8_t key[16];

    /* ID3 timestamp handling (elementary audio streams have ID3 timestamps
     * (and possibly other ID3 tags) in the beginning of each segment) */
    int is_id3_timestamped; /* -1: not yet known */
    int64_t id3_mpegts_timestamp; /* in mpegts tb */
    int64_t id3_offset; /* in stream original tb */
    uint8_t* id3_buf; /* temp buffer for id3 parsing */
    unsigned int id3_buf_size;
    AVDictionary *id3_initial; /* data from first id3 tag */
    int id3_found; /* ID3 tag found at some point */
    int id3_changed; /* ID3 tag data has changed at some point */
    ID3v2ExtraMeta *id3_deferred_extra; /* stored here until subdemuxer is opened */

    HLSAudioSetupInfo audio_setup_info;

    int64_t seek_timestamp;
    int seek_flags;
    int seek_stream_index; /* into subdemuxer stream array */

    /* Renditions associated with this playlist, if any.
     * Alternative rendition playlists have a single rendition associated
     * with them, and variant main Media Playlists may have
     * multiple (playlist-less) renditions associated with them. */
    int n_renditions;
    struct rendition **renditions;

    /* Media Initialization Sections (EXT-X-MAP) associated with this
     * playlist, if any. */
    int n_init_sections;
    struct segment **init_sections;
};

/*
 * Renditions are e.g. alternative subtitle or audio streams.
 * The rendition may either be an external playlist or it may be
 * contained in the main Media Playlist of the variant (in which case
 * playlist is NULL).
 */
struct rendition {
    enum AVMediaType type;
    struct playlist *playlist;
    char group_id[MAX_FIELD_LEN];
    char language[MAX_FIELD_LEN];
    char name[MAX_FIELD_LEN];
    int disposition;
};

struct variant {
    int bandwidth;

    /* every variant contains at least the main Media Playlist in index 0 */
    int n_playlists;
    struct playlist **playlists;

    char audio_group[MAX_FIELD_LEN];
    char video_group[MAX_FIELD_LEN];
    char subtitles_group[MAX_FIELD_LEN];
};

typedef struct HLSContext {
    AVClass *class;
    AVFormatContext *ctx;
    int n_variants;
    struct variant **variants;
    int n_playlists;
    struct playlist **playlists;
    int n_renditions;
    struct rendition **renditions;

    int64_t cur_seq_no;
    int m3u8_hold_counters;
    int live_start_index;
    int prefer_x_start;
    int first_packet;
    int64_t first_timestamp;
    int64_t cur_timestamp;
    AVIOInterruptCB *interrupt_callback;
    AVDictionary *avio_opts;
    AVDictionary *seg_format_opts;
    char *allowed_extensions;
    char *allowed_segment_extensions;
    int extension_picky;
    int max_reload;
    int http_persistent;
    int http_multiple;
    int http_seekable;
    int seg_max_retry;
    AVIOContext *playlist_pb;
    HLSCryptoContext  crypto_ctx;
} HLSContext;

static void free_segment_dynarray(struct segment **segments, int n_segments)
{
    int i;
    for (i = 0; i < n_segments; i++) {
        av_freep(&segments[i]->key);
        av_freep(&segments[i]->url);
        av_freep(&segments[i]);
    }
}

static void free_segment_list(struct playlist *pls)
{
    free_segment_dynarray(pls->segments, pls->n_segments);
    av_freep(&pls->segments);
    pls->n_segments = 0;
}

static void free_init_section_list(struct playlist *pls)
{
    int i;
    for (i = 0; i < pls->n_init_sections; i++) {
        av_freep(&pls->init_sections[i]->key);
        av_freep(&pls->init_sections[i]->url);
        av_freep(&pls->init_sections[i]);
    }
    av_freep(&pls->init_sections);
    pls->n_init_sections = 0;
}

static void free_playlist_list(HLSContext *c)
{
    int i;
    for (i = 0; i < c->n_playlists; i++) {
        struct playlist *pls = c->playlists[i];
        free_segment_list(pls);
        free_init_section_list(pls);
        av_freep(&pls->main_streams);
        av_freep(&pls->renditions);
        av_freep(&pls->id3_buf);
        av_dict_free(&pls->id3_initial);
        ff_id3v2_free_extra_meta(&pls->id3_deferred_extra);
        av_freep(&pls->init_sec_buf);
        av_packet_free(&pls->pkt);
        av_freep(&pls->pb.pub.buffer);
        ff_format_io_close(c->ctx, &pls->input);
        pls->input_read_done = 0;
        ff_format_io_close(c->ctx, &pls->input_next);
        pls->input_next_requested = 0;
        if (pls->ctx) {
            pls->ctx->pb = NULL;
            avformat_close_input(&pls->ctx);
        }
        av_free(pls);
    }
    av_freep(&c->playlists);
    c->n_playlists = 0;
}

static void free_variant_list(HLSContext *c)
{
    int i;
    for (i = 0; i < c->n_variants; i++) {
        struct variant *var = c->variants[i];
        av_freep(&var->playlists);
        av_free(var);
    }
    av_freep(&c->variants);
    c->n_variants = 0;
}

static void free_rendition_list(HLSContext *c)
{
    int i;
    for (i = 0; i < c->n_renditions; i++)
        av_freep(&c->renditions[i]);
    av_freep(&c->renditions);
    c->n_renditions = 0;
}

static struct playlist *new_playlist(HLSContext *c, const char *url,
                                     const char *base)
{
    struct playlist *pls = av_mallocz(sizeof(struct playlist));
    if (!pls)
        return NULL;
    pls->pkt = av_packet_alloc();
    if (!pls->pkt) {
        av_free(pls);
        return NULL;
    }
    ff_make_absolute_url(pls->url, sizeof(pls->url), base, url);
    if (!pls->url[0]) {
        av_packet_free(&pls->pkt);
        av_free(pls);
        return NULL;
    }
    pls->seek_timestamp = AV_NOPTS_VALUE;

    pls->is_id3_timestamped = -1;
    pls->id3_mpegts_timestamp = AV_NOPTS_VALUE;

    dynarray_add(&c->playlists, &c->n_playlists, pls);
    return pls;
}

struct variant_info {
    char bandwidth[20];
    /* variant group ids: */
    char audio[MAX_FIELD_LEN];
    char video[MAX_FIELD_LEN];
    char subtitles[MAX_FIELD_LEN];
};

static struct variant *new_variant(HLSContext *c, struct variant_info *info,
                                   const char *url, const char *base)
{
    struct variant *var;
    struct playlist *pls;

    pls = new_playlist(c, url, base);
    if (!pls)
        return NULL;

    var = av_mallocz(sizeof(struct variant));
    if (!var)
        return NULL;

    if (info) {
        var->bandwidth = atoi(info->bandwidth);
        strcpy(var->audio_group, info->audio);
        strcpy(var->video_group, info->video);
        strcpy(var->subtitles_group, info->subtitles);
    }

    dynarray_add(&c->variants, &c->n_variants, var);
    dynarray_add(&var->playlists, &var->n_playlists, pls);
    return var;
}

static void handle_variant_args(struct variant_info *info, const char *key,
                                int key_len, char **dest, int *dest_len)
{
    if (!strncmp(key, "BANDWIDTH=", key_len)) {
        *dest     =        info->bandwidth;
        *dest_len = sizeof(info->bandwidth);
    } else if (!strncmp(key, "AUDIO=", key_len)) {
        *dest     =        info->audio;
        *dest_len = sizeof(info->audio);
    } else if (!strncmp(key, "VIDEO=", key_len)) {
        *dest     =        info->video;
        *dest_len = sizeof(info->video);
    } else if (!strncmp(key, "SUBTITLES=", key_len)) {
        *dest     =        info->subtitles;
        *dest_len = sizeof(info->subtitles);
    }
}

struct key_info {
     char uri[MAX_URL_SIZE];
     char method[11];
     char iv[35];
};

static void handle_key_args(struct key_info *info, const char *key,
                            int key_len, char **dest, int *dest_len)
{
    if (!strncmp(key, "METHOD=", key_len)) {
        *dest     =        info->method;
        *dest_len = sizeof(info->method);
    } else if (!strncmp(key, "URI=", key_len)) {
        *dest     =        info->uri;
        *dest_len = sizeof(info->uri);
    } else if (!strncmp(key, "IV=", key_len)) {
        *dest     =        info->iv;
        *dest_len = sizeof(info->iv);
    }
}

struct init_section_info {
    char uri[MAX_URL_SIZE];
    char byterange[32];
};

static struct segment *new_init_section(struct playlist *pls,
                                        struct init_section_info *info,
                                        const char *url_base)
{
    struct segment *sec;
    char tmp_str[MAX_URL_SIZE], *ptr = tmp_str;

    if (!info->uri[0])
        return NULL;

    sec = av_mallocz(sizeof(*sec));
    if (!sec)
        return NULL;

    if (!av_strncasecmp(info->uri, "data:", 5)) {
        ptr = info->uri;
    } else {
        ff_make_absolute_url(tmp_str, sizeof(tmp_str), url_base, info->uri);
        if (!tmp_str[0]) {
            av_free(sec);
            return NULL;
        }
    }
    sec->url = av_strdup(ptr);
    if (!sec->url) {
        av_free(sec);
        return NULL;
    }

    if (info->byterange[0]) {
        sec->size = strtoll(info->byterange, NULL, 10);
        ptr = strchr(info->byterange, '@');
        if (ptr)
            sec->url_offset = strtoll(ptr+1, NULL, 10);
    } else {
        /* the entire file is the init section */
        sec->size = -1;
    }

    dynarray_add(&pls->init_sections, &pls->n_init_sections, sec);

    return sec;
}

static void handle_init_section_args(struct init_section_info *info, const char *key,
                                           int key_len, char **dest, int *dest_len)
{
    if (!strncmp(key, "URI=", key_len)) {
        *dest     =        info->uri;
        *dest_len = sizeof(info->uri);
    } else if (!strncmp(key, "BYTERANGE=", key_len)) {
        *dest     =        info->byterange;
        *dest_len = sizeof(info->byterange);
    }
}

struct rendition_info {
    char type[16];
    char uri[MAX_URL_SIZE];
    char group_id[MAX_FIELD_LEN];
    char language[MAX_FIELD_LEN];
    char assoc_language[MAX_FIELD_LEN];
    char name[MAX_FIELD_LEN];
    char defaultr[4];
    char forced[4];
    char characteristics[MAX_CHARACTERISTICS_LEN];
};

static struct rendition *new_rendition(HLSContext *c, struct rendition_info *info,
                                      const char *url_base)
{
    struct rendition *rend;
    enum AVMediaType type = AVMEDIA_TYPE_UNKNOWN;
    char *characteristic;
    char *chr_ptr;
    char *saveptr;

    if (!strcmp(info->type, "AUDIO"))
        type = AVMEDIA_TYPE_AUDIO;
    else if (!strcmp(info->type, "VIDEO"))
        type = AVMEDIA_TYPE_VIDEO;
    else if (!strcmp(info->type, "SUBTITLES"))
        type = AVMEDIA_TYPE_SUBTITLE;
    else if (!strcmp(info->type, "CLOSED-CAPTIONS"))
        /* CLOSED-CAPTIONS is ignored since we do not support CEA-608 CC in
         * AVC SEI RBSP anyway */
        return NULL;

    if (type == AVMEDIA_TYPE_UNKNOWN) {
        av_log(c->ctx, AV_LOG_WARNING, "Can't support the type: %s\n", info->type);
        return NULL;
    }

    /* URI is mandatory for subtitles as per spec */
    if (type == AVMEDIA_TYPE_SUBTITLE && !info->uri[0]) {
        av_log(c->ctx, AV_LOG_ERROR, "The URI tag is REQUIRED for subtitle.\n");
        return NULL;
    }

    /* TODO: handle subtitles (each segment has to parsed separately) */
    if (c->ctx->strict_std_compliance > FF_COMPLIANCE_EXPERIMENTAL)
        if (type == AVMEDIA_TYPE_SUBTITLE) {
            av_log(c->ctx, AV_LOG_WARNING, "Can't support the subtitle(uri: %s)\n", info->uri);
            return NULL;
        }

    rend = av_mallocz(sizeof(struct rendition));
    if (!rend)
        return NULL;

    dynarray_add(&c->renditions, &c->n_renditions, rend);

    rend->type = type;
    strcpy(rend->group_id, info->group_id);
    strcpy(rend->language, info->language);
    strcpy(rend->name, info->name);

    /* add the playlist if this is an external rendition */
    if (info->uri[0]) {
        rend->playlist = new_playlist(c, info->uri, url_base);
        if (rend->playlist)
            dynarray_add(&rend->playlist->renditions,
                         &rend->playlist->n_renditions, rend);
    }

    if (info->assoc_language[0]) {
        size_t langlen = strlen(rend->language);
        if (langlen < sizeof(rend->language) - 3) {
            size_t assoc_len;
            rend->language[langlen] = ',';
            assoc_len = av_strlcpy(rend->language + langlen + 1,
                                   info->assoc_language,
                                   sizeof(rend->language) - langlen - 1);
            if (langlen + assoc_len + 2 > sizeof(rend->language)) // truncation occurred
                av_log(c->ctx, AV_LOG_WARNING, "Truncated rendition language: %s\n",
                       info->assoc_language);
        }
    }

    if (!strcmp(info->defaultr, "YES"))
        rend->disposition |= AV_DISPOSITION_DEFAULT;
    if (!strcmp(info->forced, "YES"))
        rend->disposition |= AV_DISPOSITION_FORCED;

    chr_ptr = info->characteristics;
    while ((characteristic = av_strtok(chr_ptr, ",", &saveptr))) {
        if (!strcmp(characteristic, "public.accessibility.describes-music-and-sound"))
            rend->disposition |= AV_DISPOSITION_HEARING_IMPAIRED;
        else if (!strcmp(characteristic, "public.accessibility.describes-video"))
            rend->disposition |= AV_DISPOSITION_VISUAL_IMPAIRED;

        chr_ptr = NULL;
    }

    return rend;
}

static void handle_rendition_args(struct rendition_info *info, const char *key,
                                  int key_len, char **dest, int *dest_len)
{
    if (!strncmp(key, "TYPE=", key_len)) {
        *dest     =        info->type;
        *dest_len = sizeof(info->type);
    } else if (!strncmp(key, "URI=", key_len)) {
        *dest     =        info->uri;
        *dest_len = sizeof(info->uri);
    } else if (!strncmp(key, "GROUP-ID=", key_len)) {
        *dest     =        info->group_id;
        *dest_len = sizeof(info->group_id);
    } else if (!strncmp(key, "LANGUAGE=", key_len)) {
        *dest     =        info->language;
        *dest_len = sizeof(info->language);
    } else if (!strncmp(key, "ASSOC-LANGUAGE=", key_len)) {
        *dest     =        info->assoc_language;
        *dest_len = sizeof(info->assoc_language);
    } else if (!strncmp(key, "NAME=", key_len)) {
        *dest     =        info->name;
        *dest_len = sizeof(info->name);
    } else if (!strncmp(key, "DEFAULT=", key_len)) {
        *dest     =        info->defaultr;
        *dest_len = sizeof(info->defaultr);
    } else if (!strncmp(key, "FORCED=", key_len)) {
        *dest     =        info->forced;
        *dest_len = sizeof(info->forced);
    } else if (!strncmp(key, "CHARACTERISTICS=", key_len)) {
        *dest     =        info->characteristics;
        *dest_len = sizeof(info->characteristics);
    }
    /*
     * ignored:
     * - AUTOSELECT: client may autoselect based on e.g. system language
     * - INSTREAM-ID: EIA-608 closed caption number ("CC1".."CC4")
     */
}

/* used by parse_playlist to allocate a new variant+playlist when the
 * playlist is detected to be a Media Playlist (not Master Playlist)
 * and we have no parent Master Playlist (parsing of which would have
 * allocated the variant and playlist already)
 * *pls == NULL  => Master Playlist or parentless Media Playlist
 * *pls != NULL => parented Media Playlist, playlist+variant allocated */
static int ensure_playlist(HLSContext *c, struct playlist **pls, const char *url)
{
    if (*pls)
        return 0;
    if (!new_variant(c, NULL, url, NULL))
        return AVERROR(ENOMEM);
    *pls = c->playlists[c->n_playlists - 1];
    return 0;
}

static int open_url_keepalive(AVFormatContext *s, AVIOContext **pb,
                              const char *url, AVDictionary **options)
{
#if !CONFIG_HTTP_PROTOCOL
    return AVERROR_PROTOCOL_NOT_FOUND;
#else
    int ret;
    URLContext *uc = ffio_geturlcontext(*pb);
    av_assert0(uc);
    (*pb)->eof_reached = 0;
    ret = ff_http_do_new_request2(uc, url, options);
    if (ret < 0) {
        ff_format_io_close(s, pb);
    }
    return ret;
#endif
}

static int open_url(AVFormatContext *s, AVIOContext **pb, const char *url,
                    AVDictionary **opts, AVDictionary *opts2, int *is_http_out)
{
    HLSContext *c = s->priv_data;
    AVDictionary *tmp = NULL;
    const char *proto_name = NULL;
    int ret;
    int is_http = 0;

    if (av_strstart(url, "crypto", NULL)) {
        if (url[6] == '+' || url[6] == ':')
            proto_name = avio_find_protocol_name(url + 7);
    } else if (av_strstart(url, "data", NULL)) {
        if (url[4] == '+' || url[4] == ':')
            proto_name = avio_find_protocol_name(url + 5);
    }

    if (!proto_name)
        proto_name = avio_find_protocol_name(url);

    if (!proto_name)
        return AVERROR_INVALIDDATA;

    // only http(s) & file are allowed
    if (av_strstart(proto_name, "file", NULL)) {
        if (strcmp(c->allowed_extensions, "ALL") && !av_match_ext(url, c->allowed_extensions)) {
            av_log(s, AV_LOG_ERROR,
                "Filename extension of \'%s\' is not a common multimedia extension, blocked for security reasons.\n"
                "If you wish to override this adjust allowed_extensions, you can set it to \'ALL\' to allow all\n",
                url);
            return AVERROR_INVALIDDATA;
        }
    } else if (av_strstart(proto_name, "http", NULL)) {
        is_http = 1;
    } else if (av_strstart(proto_name, "data", NULL)) {
        ;
    } else
        return AVERROR_INVALIDDATA;

    if (!strncmp(proto_name, url, strlen(proto_name)) && url[strlen(proto_name)] == ':')
        ;
    else if (av_strstart(url, "crypto", NULL) && !strncmp(proto_name, url + 7, strlen(proto_name)) && url[7 + strlen(proto_name)] == ':')
        ;
    else if (av_strstart(url, "data", NULL) && !strncmp(proto_name, url + 5, strlen(proto_name)) && url[5 + strlen(proto_name)] == ':')
        ;
    else if (strcmp(proto_name, "file") || !strncmp(url, "file,", 5))
        return AVERROR_INVALIDDATA;

    av_dict_copy(&tmp, *opts, 0);
    av_dict_copy(&tmp, opts2, 0);

    if (is_http && c->http_persistent && *pb) {
        ret = open_url_keepalive(c->ctx, pb, url, &tmp);
        if (ret == AVERROR_EXIT) {
            av_dict_free(&tmp);
            return ret;
        } else if (ret < 0) {
            if (ret != AVERROR_EOF)
                av_log(s, AV_LOG_WARNING,
                    "keepalive request failed for '%s' with error: '%s' when opening url, retrying with new connection\n",
                    url, av_err2str(ret));
            av_dict_copy(&tmp, *opts, 0);
            av_dict_copy(&tmp, opts2, 0);
            ret = s->io_open(s, pb, url, AVIO_FLAG_READ, &tmp);
        }
    } else {
        ret = s->io_open(s, pb, url, AVIO_FLAG_READ, &tmp);
    }
    if (ret >= 0) {
        // update cookies on http response with setcookies.
        char *new_cookies = NULL;

        if (!(s->flags & AVFMT_FLAG_CUSTOM_IO))
            av_opt_get(*pb, "cookies", AV_OPT_SEARCH_CHILDREN, (uint8_t**)&new_cookies);

        if (new_cookies)
            av_dict_set(opts, "cookies", new_cookies, AV_DICT_DONT_STRDUP_VAL);
    }

    av_dict_free(&tmp);

    if (is_http_out)
        *is_http_out = is_http;

    return ret;
}

static int test_segment(AVFormatContext *s, const AVInputFormat *in_fmt, struct playlist *pls, struct segment *seg)
{
    HLSContext *c = s->priv_data;
    int matchA = 3;
    int matchF = 0;

    if (!c->extension_picky)
        return 0;

    if (strcmp(c->allowed_segment_extensions, "ALL"))
        matchA =      av_match_ext    (seg->url, c->allowed_segment_extensions)
                 + 2*(ff_match_url_ext(seg->url, c->allowed_segment_extensions) > 0);

    if (!matchA) {
        av_log(s, AV_LOG_ERROR, "URL %s is not in allowed_segment_extensions\n", seg->url);
        return AVERROR_INVALIDDATA;
    }

    if (in_fmt) {
        if (in_fmt->extensions) {
            matchF =      av_match_ext(    seg->url, in_fmt->extensions)
                     + 2*(ff_match_url_ext(seg->url, in_fmt->extensions) > 0);
            // Youtube uses aac files with .ts extension
            if(av_match_name("mp4", in_fmt->name) || av_match_name("aac", in_fmt->name)) {
                matchF |=      av_match_ext(    seg->url, "ts,m2t,m2ts,mts,mpg,m4s,mpeg,mpegts")
                          + 2*(ff_match_url_ext(seg->url, "ts,m2t,m2ts,mts,mpg,m4s,mpeg,mpegts") > 0);
            }
        } else if (!strcmp(in_fmt->name, "mpegts")) {
            const char *str = "ts,m2t,m2ts,mts,mpg,m4s,mpeg,mpegts"
                              ",html" // https://flash1.bogulus.cfd/
                            ;
            matchF =      av_match_ext(    seg->url, str)
                     + 2*(ff_match_url_ext(seg->url, str) > 0);
        }

        if (!(matchA & matchF)) {
            av_log(s, AV_LOG_ERROR, "detected format %s extension %s mismatches allowed extensions in url %s\n", in_fmt->name, in_fmt->extensions ? in_fmt->extensions : "none", seg->url);
            return AVERROR_INVALIDDATA;
        }
    }

    return 0;
}

static int parse_playlist(HLSContext *c, const char *url,
                          struct playlist *pls, AVIOContext *in)
{
    int ret = 0, is_segment = 0, is_variant = 0;
    int64_t duration = 0;
    enum KeyType key_type = KEY_NONE;
    uint8_t iv[16] = "";
    int has_iv = 0;
    char key[MAX_URL_SIZE] = "";
    char line[MAX_URL_SIZE];
    const char *ptr;
    int close_in = 0;
    int64_t seg_offset = 0;
    int64_t seg_size = -1;
    uint8_t *new_url = NULL;
    struct variant_info variant_info;
    char tmp_str[MAX_URL_SIZE];
    struct segment *cur_init_section = NULL;
    int is_http = av_strstart(url, "http", NULL);
    struct segment **prev_segments = NULL;
    int prev_n_segments = 0;
    int64_t prev_start_seq_no = -1;

    if (is_http && !in && c->http_persistent && c->playlist_pb) {
        in = c->playlist_pb;
        ret = open_url_keepalive(c->ctx, &c->playlist_pb, url, NULL);
        if (ret == AVERROR_EXIT) {
            return ret;
        } else if (ret < 0) {
            if (ret != AVERROR_EOF)
                av_log(c->ctx, AV_LOG_WARNING,
                    "keepalive request failed for '%s' with error: '%s' when parsing playlist\n",
                    url, av_err2str(ret));
            in = NULL;
        }
    }

    if (!in) {
        AVDictionary *opts = NULL;
        av_dict_copy(&opts, c->avio_opts, 0);

        if (c->http_persistent)
            av_dict_set(&opts, "multiple_requests", "1", 0);

        ret = c->ctx->io_open(c->ctx, &in, url, AVIO_FLAG_READ, &opts);
        av_dict_free(&opts);
        if (ret < 0)
            return ret;

        if (is_http && c->http_persistent)
            c->playlist_pb = in;
        else
            close_in = 1;
    }

    if (av_opt_get(in, "location", AV_OPT_SEARCH_CHILDREN, &new_url) >= 0)
        url = new_url;

    ff_get_chomp_line(in, line, sizeof(line));
    if (strcmp(line, "#EXTM3U")) {
        ret = AVERROR_INVALIDDATA;
        goto fail;
    }

    if (pls) {
        prev_start_seq_no = pls->start_seq_no;
        prev_segments = pls->segments;
        prev_n_segments = pls->n_segments;
        pls->segments = NULL;
        pls->n_segments = 0;

        pls->finished = 0;
        pls->type = PLS_TYPE_UNSPECIFIED;
    }
    while (!avio_feof(in)) {
        ff_get_chomp_line(in, line, sizeof(line));
        if (av_strstart(line, "#EXT-X-STREAM-INF:", &ptr)) {
            is_variant = 1;
            memset(&variant_info, 0, sizeof(variant_info));
            ff_parse_key_value(ptr, (ff_parse_key_val_cb) handle_variant_args,
                               &variant_info);
        } else if (av_strstart(line, "#EXT-X-KEY:", &ptr)) {
            struct key_info info = {{0}};
            ff_parse_key_value(ptr, (ff_parse_key_val_cb) handle_key_args,
                               &info);
            key_type = KEY_NONE;
            has_iv = 0;
            if (!strcmp(info.method, "AES-128"))
                key_type = KEY_AES_128;
            if (!strcmp(info.method, "SAMPLE-AES"))
                key_type = KEY_SAMPLE_AES;
            if (!av_strncasecmp(info.iv, "0x", 2)) {
                ff_hex_to_data(iv, info.iv + 2);
                has_iv = 1;
            }
            av_strlcpy(key, info.uri, sizeof(key));
        } else if (av_strstart(line, "#EXT-X-MEDIA:", &ptr)) {
            struct rendition_info info = {{0}};
            ff_parse_key_value(ptr, (ff_parse_key_val_cb) handle_rendition_args,
                               &info);
            new_rendition(c, &info, url);
        } else if (av_strstart(line, "#EXT-X-TARGETDURATION:", &ptr)) {
            int64_t t;
            ret = ensure_playlist(c, &pls, url);
            if (ret < 0)
                goto fail;
            t = strtoll(ptr, NULL, 10);
            if (t < 0 || t >= INT64_MAX / AV_TIME_BASE) {
                ret = AVERROR_INVALIDDATA;
                goto fail;
            }
            pls->target_duration = t * AV_TIME_BASE;
        } else if (av_strstart(line, "#EXT-X-MEDIA-SEQUENCE:", &ptr)) {
            uint64_t seq_no;
            ret = ensure_playlist(c, &pls, url);
            if (ret < 0)
                goto fail;
            seq_no = strtoull(ptr, NULL, 10);
            if (seq_no > INT64_MAX/2) {
                av_log(c->ctx, AV_LOG_DEBUG, "MEDIA-SEQUENCE higher than "
                        "INT64_MAX/2, mask out the highest bit\n");
                seq_no &= INT64_MAX/2;
            }
            pls->start_seq_no = seq_no;
        } else if (av_strstart(line, "#EXT-X-PLAYLIST-TYPE:", &ptr)) {
            ret = ensure_playlist(c, &pls, url);
            if (ret < 0)
                goto fail;
            if (!strcmp(ptr, "EVENT"))
                pls->type = PLS_TYPE_EVENT;
            else if (!strcmp(ptr, "VOD"))
                pls->type = PLS_TYPE_VOD;
        } else if (av_strstart(line, "#EXT-X-MAP:", &ptr)) {
            struct init_section_info info = {{0}};
            ret = ensure_playlist(c, &pls, url);
            if (ret < 0)
                goto fail;
            ff_parse_key_value(ptr, (ff_parse_key_val_cb) handle_init_section_args,
                               &info);
            cur_init_section = new_init_section(pls, &info, url);
            if (!cur_init_section) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }
            cur_init_section->key_type = key_type;
            if (has_iv) {
                memcpy(cur_init_section->iv, iv, sizeof(iv));
            } else {
                int64_t seq = pls->start_seq_no + pls->n_segments;
                memset(cur_init_section->iv, 0, sizeof(cur_init_section->iv));
                AV_WB64(cur_init_section->iv + 8, seq);
            }

            if (key_type != KEY_NONE) {
                ff_make_absolute_url(tmp_str, sizeof(tmp_str), url, key);
                if (!tmp_str[0]) {
                    av_free(cur_init_section);
                    ret = AVERROR_INVALIDDATA;
                    goto fail;
                }
                cur_init_section->key = av_strdup(tmp_str);
                if (!cur_init_section->key) {
                    av_free(cur_init_section);
                    ret = AVERROR(ENOMEM);
                    goto fail;
                }
            } else {
                cur_init_section->key = NULL;
            }

        } else if (av_strstart(line, "#EXT-X-START:", &ptr)) {
            const char *time_offset_value = NULL;
            ret = ensure_playlist(c, &pls, url);
            if (ret < 0) {
                goto fail;
            }
            if (av_strstart(ptr, "TIME-OFFSET=", &time_offset_value)) {
                float offset = strtof(time_offset_value, NULL);
                pls->start_time_offset = offset * AV_TIME_BASE;
                pls->time_offset_flag = 1;
            } else {
                av_log(c->ctx, AV_LOG_WARNING, "#EXT-X-START value is"
                                                "invalid, it will be ignored");
                continue;
            }
        } else if (av_strstart(line, "#EXT-X-ENDLIST", &ptr)) {
            if (pls)
                pls->finished = 1;
        } else if (av_strstart(line, "#EXTINF:", &ptr)) {
            is_segment = 1;
            duration   = atof(ptr) * AV_TIME_BASE;
        } else if (av_strstart(line, "#EXT-X-BYTERANGE:", &ptr)) {
            seg_size = strtoll(ptr, NULL, 10);
            ptr = strchr(ptr, '@');
            if (ptr)
                seg_offset = strtoll(ptr+1, NULL, 10);
        } else if (av_strstart(line, "#", NULL)) {
            av_log(c->ctx, AV_LOG_INFO, "Skip ('%s')\n", line);
            continue;
        } else if (line[0]) {
            if (is_variant) {
                if (!new_variant(c, &variant_info, line, url)) {
                    ret = AVERROR(ENOMEM);
                    goto fail;
                }
                is_variant = 0;
            }
            if (is_segment) {
                struct segment *seg;
                ret = ensure_playlist(c, &pls, url);
                if (ret < 0)
                    goto fail;
                seg = av_malloc(sizeof(struct segment));
                if (!seg) {
                    ret = AVERROR(ENOMEM);
                    goto fail;
                }
                if (has_iv) {
                    memcpy(seg->iv, iv, sizeof(iv));
                } else {
                    uint64_t seq = pls->start_seq_no + (uint64_t)pls->n_segments;
                    memset(seg->iv, 0, sizeof(seg->iv));
                    AV_WB64(seg->iv + 8, seq);
                }

                if (key_type != KEY_NONE) {
                    ff_make_absolute_url(tmp_str, sizeof(tmp_str), url, key);
                    if (!tmp_str[0]) {
                        ret = AVERROR_INVALIDDATA;
                        av_free(seg);
                        goto fail;
                    }
                    seg->key = av_strdup(tmp_str);
                    if (!seg->key) {
                        av_free(seg);
                        ret = AVERROR(ENOMEM);
                        goto fail;
                    }
                } else {
                    seg->key = NULL;
                }

                ff_make_absolute_url(tmp_str, sizeof(tmp_str), url, line);
                if (!tmp_str[0]) {
                    ret = AVERROR_INVALIDDATA;
                    if (seg->key)
                        av_free(seg->key);
                    av_free(seg);
                    goto fail;
                }
                seg->url = av_strdup(tmp_str);
                if (!seg->url) {
                    av_free(seg->key);
                    av_free(seg);
                    ret = AVERROR(ENOMEM);
                    goto fail;
                }

                ret = test_segment(c->ctx, pls->ctx ? pls->ctx->iformat : NULL, pls, seg);
                if (ret < 0) {
                    av_free(seg->url);
                    av_free(seg->key);
                    av_free(seg);
                    goto fail;
                }

                if (duration < 0.001 * AV_TIME_BASE) {
                    av_log(c->ctx, AV_LOG_WARNING, "Cannot get correct #EXTINF value of segment %s,"
                                    " set to default value to 1ms.\n", seg->url);
                    duration = 0.001 * AV_TIME_BASE;
                }
                seg->duration = duration;
                seg->key_type = key_type;
                dynarray_add(&pls->segments, &pls->n_segments, seg);
                is_segment = 0;

                seg->size = seg_size;
                if (seg_size >= 0) {
                    seg->url_offset = seg_offset;
                    seg_offset += seg_size;
                    seg_size = -1;
                } else {
                    seg->url_offset = 0;
                    seg_offset = 0;
                }

                seg->init_section = cur_init_section;
            }
        }
    }
    if (prev_segments) {
        if (pls->start_seq_no > prev_start_seq_no && c->first_timestamp != AV_NOPTS_VALUE) {
            int64_t prev_timestamp = c->first_timestamp;
            int i;
            int64_t diff = pls->start_seq_no - prev_start_seq_no;
            for (i = 0; i < prev_n_segments && i < diff; i++) {
                c->first_timestamp += prev_segments[i]->duration;
            }
            av_log(c->ctx, AV_LOG_DEBUG, "Media sequence change (%"PRId64" -> %"PRId64")"
                   " reflected in first_timestamp: %"PRId64" -> %"PRId64"\n",
                   prev_start_seq_no, pls->start_seq_no,
                   prev_timestamp, c->first_timestamp);
        } else if (pls->start_seq_no < prev_start_seq_no) {
            av_log(c->ctx, AV_LOG_WARNING, "Media sequence changed unexpectedly: %"PRId64" -> %"PRId64"\n",
                   prev_start_seq_no, pls->start_seq_no);
        }
        free_segment_dynarray(prev_segments, prev_n_segments);
        av_freep(&prev_segments);
    }
    if (pls)
        pls->last_load_time = av_gettime_relative();

fail:
    av_free(new_url);
    if (close_in)
        ff_format_io_close(c->ctx, &in);
    c->ctx->ctx_flags = c->ctx->ctx_flags & ~(unsigned)AVFMTCTX_UNSEEKABLE;
    if (!c->n_variants || !c->variants[0]->n_playlists ||
        !(c->variants[0]->playlists[0]->finished ||
          c->variants[0]->playlists[0]->type == PLS_TYPE_EVENT))
        c->ctx->ctx_flags |= AVFMTCTX_UNSEEKABLE;
    return ret;
}

static struct segment *current_segment(struct playlist *pls)
{
    int64_t n = pls->cur_seq_no - pls->start_seq_no;
    if (n >= pls->n_segments)
        return NULL;
    return pls->segments[n];
}

static struct segment *next_segment(struct playlist *pls)
{
    int64_t n = pls->cur_seq_no - pls->start_seq_no + 1;
    if (n >= pls->n_segments)
        return NULL;
    return pls->segments[n];
}

static int read_from_url(struct playlist *pls, struct segment *seg,
                         uint8_t *buf, int buf_size)
{
    int ret;

     /* limit read if the segment was only a part of a file */
    if (seg->size >= 0)
        buf_size = FFMIN(buf_size, seg->size - pls->cur_seg_offset);

    ret = avio_read(pls->input, buf, buf_size);
    if (ret > 0)
        pls->cur_seg_offset += ret;

    return ret;
}

/* Parse the raw ID3 data and pass contents to caller */
static void parse_id3(AVFormatContext *s, AVIOContext *pb,
                      AVDictionary **metadata, int64_t *dts, HLSAudioSetupInfo *audio_setup_info,
                      ID3v2ExtraMetaAPIC **apic, ID3v2ExtraMeta **extra_meta)
{
    static const char id3_priv_owner_ts[] = "com.apple.streaming.transportStreamTimestamp";
    static const char id3_priv_owner_audio_setup[] = "com.apple.streaming.audioDescription";
    ID3v2ExtraMeta *meta;

    ff_id3v2_read_dict(pb, metadata, ID3v2_DEFAULT_MAGIC, extra_meta);
    for (meta = *extra_meta; meta; meta = meta->next) {
        if (!strcmp(meta->tag, "PRIV")) {
            ID3v2ExtraMetaPRIV *priv = &meta->data.priv;
            if (priv->datasize == 8 && !av_strncasecmp(priv->owner, id3_priv_owner_ts, 44)) {
                /* 33-bit MPEG timestamp */
                int64_t ts = AV_RB64(priv->data);
                av_log(s, AV_LOG_DEBUG, "HLS ID3 audio timestamp %"PRId64"\n", ts);
                if ((ts & ~((1ULL << 33) - 1)) == 0)
                    *dts = ts;
                else
                    av_log(s, AV_LOG_ERROR, "Invalid HLS ID3 audio timestamp %"PRId64"\n", ts);
            } else if (priv->datasize >= 8 && !av_strncasecmp(priv->owner, id3_priv_owner_audio_setup, 36)) {
                ff_hls_senc_read_audio_setup_info(audio_setup_info, priv->data, priv->datasize);
            }
        } else if (!strcmp(meta->tag, "APIC") && apic)
            *apic = &meta->data.apic;
    }
}

/* Check if the ID3 metadata contents have changed */
static int id3_has_changed_values(struct playlist *pls, AVDictionary *metadata,
                                  ID3v2ExtraMetaAPIC *apic)
{
    const AVDictionaryEntry *entry = NULL;
    const AVDictionaryEntry *oldentry;
    /* check that no keys have changed values */
    while ((entry = av_dict_iterate(metadata, entry))) {
        oldentry = av_dict_get(pls->id3_initial, entry->key, NULL, AV_DICT_MATCH_CASE);
        if (!oldentry || strcmp(oldentry->value, entry->value) != 0)
            return 1;
    }

    /* check if apic appeared */
    if (apic && (pls->ctx->nb_streams != 2 || !pls->ctx->streams[1]->attached_pic.data))
        return 1;

    if (apic) {
        int size = pls->ctx->streams[1]->attached_pic.size;
        if (size != apic->buf->size - AV_INPUT_BUFFER_PADDING_SIZE)
            return 1;

        if (memcmp(apic->buf->data, pls->ctx->streams[1]->attached_pic.data, size) != 0)
            return 1;
    }

    return 0;
}

/* Parse ID3 data and handle the found data */
static void handle_id3(AVIOContext *pb, struct playlist *pls)
{
    AVDictionary *metadata = NULL;
    ID3v2ExtraMetaAPIC *apic = NULL;
    ID3v2ExtraMeta *extra_meta = NULL;
    int64_t timestamp = AV_NOPTS_VALUE;

    parse_id3(pls->ctx, pb, &metadata, &timestamp, &pls->audio_setup_info, &apic, &extra_meta);

    if (timestamp != AV_NOPTS_VALUE) {
        pls->id3_mpegts_timestamp = timestamp;
        pls->id3_offset = 0;
    }

    if (!pls->id3_found) {
        /* initial ID3 tags */
        av_assert0(!pls->id3_deferred_extra);
        pls->id3_found = 1;

        /* get picture attachment and set text metadata */
        if (pls->ctx->nb_streams)
            ff_id3v2_parse_apic(pls->ctx, extra_meta);
        else
            /* demuxer not yet opened, defer picture attachment */
            pls->id3_deferred_extra = extra_meta;

        ff_id3v2_parse_priv_dict(&metadata, extra_meta);
        av_dict_copy(&pls->ctx->metadata, metadata, 0);
        pls->id3_initial = metadata;

    } else {
        if (!pls->id3_changed && id3_has_changed_values(pls, metadata, apic)) {
            avpriv_report_missing_feature(pls->parent, "Changing ID3 metadata in HLS audio elementary stream");
            pls->id3_changed = 1;
        }
        av_dict_free(&metadata);
    }

    if (!pls->id3_deferred_extra)
        ff_id3v2_free_extra_meta(&extra_meta);
}

static void intercept_id3(struct playlist *pls, uint8_t *buf,
                         int buf_size, int *len)
{
    /* intercept id3 tags, we do not want to pass them to the raw
     * demuxer on all segment switches */
    int bytes;
    int id3_buf_pos = 0;
    int fill_buf = 0;
    struct segment *seg = current_segment(pls);

    /* gather all the id3 tags */
    while (1) {
        /* see if we can retrieve enough data for ID3 header */
        if (*len < ID3v2_HEADER_SIZE && buf_size >= ID3v2_HEADER_SIZE) {
            bytes = read_from_url(pls, seg, buf + *len, ID3v2_HEADER_SIZE - *len);
            if (bytes > 0) {

                if (bytes == ID3v2_HEADER_SIZE - *len)
                    /* no EOF yet, so fill the caller buffer again after
                     * we have stripped the ID3 tags */
                    fill_buf = 1;

                *len += bytes;

            } else if (*len <= 0) {
                /* error/EOF */
                *len = bytes;
                fill_buf = 0;
            }
        }

        if (*len < ID3v2_HEADER_SIZE)
            break;

        if (ff_id3v2_match(buf, ID3v2_DEFAULT_MAGIC)) {
            int64_t maxsize = seg->size >= 0 ? seg->size : 1024*1024;
            int taglen = ff_id3v2_tag_len(buf);
            int tag_got_bytes = FFMIN(taglen, *len);
            int remaining = taglen - tag_got_bytes;

            if (taglen > maxsize) {
                av_log(pls->parent, AV_LOG_ERROR, "Too large HLS ID3 tag (%d > %"PRId64" bytes)\n",
                       taglen, maxsize);
                break;
            }

            /*
             * Copy the id3 tag to our temporary id3 buffer.
             * We could read a small id3 tag directly without memcpy, but
             * we would still need to copy the large tags, and handling
             * both of those cases together with the possibility for multiple
             * tags would make the handling a bit complex.
             */
            pls->id3_buf = av_fast_realloc(pls->id3_buf, &pls->id3_buf_size, id3_buf_pos + taglen);
            if (!pls->id3_buf)
                break;
            memcpy(pls->id3_buf + id3_buf_pos, buf, tag_got_bytes);
            id3_buf_pos += tag_got_bytes;

            /* strip the intercepted bytes */
            *len -= tag_got_bytes;
            memmove(buf, buf + tag_got_bytes, *len);
            av_log(pls->parent, AV_LOG_DEBUG, "Stripped %d HLS ID3 bytes\n", tag_got_bytes);

            if (remaining > 0) {
                /* read the rest of the tag in */
                if (read_from_url(pls, seg, pls->id3_buf + id3_buf_pos, remaining) != remaining)
                    break;
                id3_buf_pos += remaining;
                av_log(pls->parent, AV_LOG_DEBUG, "Stripped additional %d HLS ID3 bytes\n", remaining);
            }

        } else {
            /* no more ID3 tags */
            break;
        }
    }

    /* re-fill buffer for the caller unless EOF */
    if (*len >= 0 && (fill_buf || *len == 0)) {
        bytes = read_from_url(pls, seg, buf + *len, buf_size - *len);

        /* ignore error if we already had some data */
        if (bytes >= 0)
            *len += bytes;
        else if (*len == 0)
            *len = bytes;
    }

    if (pls->id3_buf) {
        /* Now parse all the ID3 tags */
        FFIOContext id3ioctx;
        ffio_init_read_context(&id3ioctx, pls->id3_buf, id3_buf_pos);
        handle_id3(&id3ioctx.pub, pls);
    }

    if (pls->is_id3_timestamped == -1)
        pls->is_id3_timestamped = (pls->id3_mpegts_timestamp != AV_NOPTS_VALUE);
}

static int open_input(HLSContext *c, struct playlist *pls, struct segment *seg, AVIOContext **in)
{
    AVDictionary *opts = NULL;
    int ret;
    int is_http = 0;

    if (c->http_persistent)
        av_dict_set(&opts, "multiple_requests", "1", 0);

    if (seg->size >= 0) {
        /* try to restrict the HTTP request to the part we want
         * (if this is in fact a HTTP request) */
        av_dict_set_int(&opts, "offset", seg->url_offset, 0);
        av_dict_set_int(&opts, "end_offset", seg->url_offset + seg->size, 0);
    }

    av_log(pls->parent, AV_LOG_VERBOSE, "HLS request for url '%s', offset %"PRId64", playlist %d\n",
           seg->url, seg->url_offset, pls->index);

    if (seg->key_type == KEY_AES_128 || seg->key_type == KEY_SAMPLE_AES) {
        if (strcmp(seg->key, pls->key_url)) {
            AVIOContext *pb = NULL;
            if (open_url(pls->parent, &pb, seg->key, &c->avio_opts, NULL, NULL) == 0) {
                ret = avio_read(pb, pls->key, sizeof(pls->key));
                if (ret != sizeof(pls->key)) {
                    av_log(pls->parent, AV_LOG_ERROR, "Unable to read key file %s\n",
                           seg->key);
                }
                ff_format_io_close(pls->parent, &pb);
            } else {
                av_log(pls->parent, AV_LOG_ERROR, "Unable to open key file %s\n",
                       seg->key);
            }
            av_strlcpy(pls->key_url, seg->key, sizeof(pls->key_url));
        }
    }

    if (seg->key_type == KEY_AES_128) {
        char iv[33], key[33], url[MAX_URL_SIZE];
        ff_data_to_hex(iv, seg->iv, sizeof(seg->iv), 0);
        ff_data_to_hex(key, pls->key, sizeof(pls->key), 0);
        if (strstr(seg->url, "://"))
            snprintf(url, sizeof(url), "crypto+%s", seg->url);
        else
            snprintf(url, sizeof(url), "crypto:%s", seg->url);

        av_dict_set(&opts, "key", key, 0);
        av_dict_set(&opts, "iv", iv, 0);

        ret = open_url(pls->parent, in, url, &c->avio_opts, opts, &is_http);
        if (ret < 0) {
            goto cleanup;
        }
        ret = 0;
    } else {
        ret = open_url(pls->parent, in, seg->url, &c->avio_opts, opts, &is_http);
    }

    /* Seek to the requested position. If this was a HTTP request, the offset
     * should already be where want it to, but this allows e.g. local testing
     * without a HTTP server.
     *
     * This is not done for HTTP at all as avio_seek() does internal bookkeeping
     * of file offset which is out-of-sync with the actual offset when "offset"
     * AVOption is used with http protocol, causing the seek to not be a no-op
     * as would be expected. Wrong offset received from the server will not be
     * noticed without the call, though.
     */
    if (ret == 0 && !is_http && seg->url_offset) {
        int64_t seekret = avio_seek(*in, seg->url_offset, SEEK_SET);
        if (seekret < 0) {
            av_log(pls->parent, AV_LOG_ERROR, "Unable to seek to offset %"PRId64" of HLS segment '%s'\n", seg->url_offset, seg->url);
            ret = seekret;
            ff_format_io_close(pls->parent, in);
        }
    }

cleanup:
    av_dict_free(&opts);
    pls->cur_seg_offset = 0;
    return ret;
}

static int update_init_section(struct playlist *pls, struct segment *seg)
{
    static const int max_init_section_size = 1024*1024;
    HLSContext *c = pls->parent->priv_data;
    int64_t sec_size;
    int64_t urlsize;
    int ret;

    if (seg->init_section == pls->cur_init_section)
        return 0;

    pls->cur_init_section = NULL;

    if (!seg->init_section)
        return 0;

    ret = open_input(c, pls, seg->init_section, &pls->input);
    if (ret < 0) {
        av_log(pls->parent, AV_LOG_WARNING,
               "Failed to open an initialization section in playlist %d\n",
               pls->index);
        return ret;
    }

    if (seg->init_section->size >= 0)
        sec_size = seg->init_section->size;
    else if ((urlsize = avio_size(pls->input)) >= 0)
        sec_size = urlsize;
    else
        sec_size = max_init_section_size;

    av_log(pls->parent, AV_LOG_DEBUG,
           "Downloading an initialization section of size %"PRId64"\n",
           sec_size);

    sec_size = FFMIN(sec_size, max_init_section_size);

    av_fast_malloc(&pls->init_sec_buf, &pls->init_sec_buf_size, sec_size);

    ret = read_from_url(pls, seg->init_section, pls->init_sec_buf,
                        pls->init_sec_buf_size);
    ff_format_io_close(pls->parent, &pls->input);

    if (ret < 0)
        return ret;

    pls->cur_init_section = seg->init_section;
    pls->init_sec_data_len = ret;
    pls->init_sec_buf_read_offset = 0;

    /* spec says audio elementary streams do not have media initialization
     * sections, so there should be no ID3 timestamps */
    pls->is_id3_timestamped = 0;

    return 0;
}

static int64_t default_reload_interval(struct playlist *pls)
{
    return pls->n_segments > 0 ?
                          pls->segments[pls->n_segments - 1]->duration :
                          pls->target_duration;
}

static int playlist_needed(struct playlist *pls)
{
    AVFormatContext *s = pls->parent;
    int i, j;
    int stream_needed = 0;
    int first_st;

    /* If there is no context or streams yet, the playlist is needed */
    if (!pls->ctx || !pls->n_main_streams)
        return 1;

    /* check if any of the streams in the playlist are needed */
    for (i = 0; i < pls->n_main_streams; i++) {
        if (pls->main_streams[i]->discard < AVDISCARD_ALL) {
            stream_needed = 1;
            break;
        }
    }

    /* If all streams in the playlist were discarded, the playlist is not
     * needed (regardless of whether whole programs are discarded or not). */
    if (!stream_needed)
        return 0;

    /* Otherwise, check if all the programs (variants) this playlist is in are
     * discarded. Since all streams in the playlist are part of the same programs
     * we can just check the programs of the first stream. */

    first_st = pls->main_streams[0]->index;

    for (i = 0; i < s->nb_programs; i++) {
        AVProgram *program = s->programs[i];
        if (program->discard < AVDISCARD_ALL) {
            for (j = 0; j < program->nb_stream_indexes; j++) {
                if (program->stream_index[j] == first_st) {
                    /* playlist is in an undiscarded program */
                    return 1;
                }
            }
        }
    }

    /* some streams were not discarded but all the programs were */
    return 0;
}

static int read_data(void *opaque, uint8_t *buf, int buf_size)
{
    struct playlist *v = opaque;
    HLSContext *c = v->parent->priv_data;
    int ret;
    int just_opened = 0;
    int reload_count = 0;
    int segment_retries = 0;
    struct segment *seg;

restart:
    if (!v->needed)
        return AVERROR_EOF;

    if (!v->input || (c->http_persistent && v->input_read_done)) {
        int64_t reload_interval;

        /* Check that the playlist is still needed before opening a new
         * segment. */
        v->needed = playlist_needed(v);

        if (!v->needed) {
            av_log(v->parent, AV_LOG_INFO, "No longer receiving playlist %d ('%s')\n",
                   v->index, v->url);
            return AVERROR_EOF;
        }

        /* If this is a live stream and the reload interval has elapsed since
         * the last playlist reload, reload the playlists now. */
        reload_interval = default_reload_interval(v);

reload:
        reload_count++;
        if (reload_count > c->max_reload)
            return AVERROR_EOF;
        if (!v->finished &&
            av_gettime_relative() - v->last_load_time >= reload_interval) {
            if ((ret = parse_playlist(c, v->url, v, NULL)) < 0) {
                if (ret != AVERROR_EXIT)
                    av_log(v->parent, AV_LOG_WARNING, "Failed to reload playlist %d\n",
                           v->index);
                return ret;
            }
            /* If we need to reload the playlist again below (if
             * there's still no more segments), switch to a reload
             * interval of half the target duration. */
            reload_interval = v->target_duration / 2;
        }
        if (v->cur_seq_no < v->start_seq_no) {
            av_log(v->parent, AV_LOG_WARNING,
                   "skipping %"PRId64" segments ahead, expired from playlists\n",
                   v->start_seq_no - v->cur_seq_no);
            v->cur_seq_no = v->start_seq_no;
        }
        if (v->cur_seq_no > v->last_seq_no) {
            v->last_seq_no = v->cur_seq_no;
            v->m3u8_hold_counters = 0;
        } else if (v->last_seq_no == v->cur_seq_no) {
            v->m3u8_hold_counters++;
            if (v->m3u8_hold_counters >= c->m3u8_hold_counters) {
                return AVERROR_EOF;
            }
        } else {
            av_log(v->parent, AV_LOG_WARNING, "The m3u8 list sequence may have been wrapped.\n");
        }
        if (v->cur_seq_no >= v->start_seq_no + v->n_segments) {
            if (v->finished)
                return AVERROR_EOF;
            while (av_gettime_relative() - v->last_load_time < reload_interval) {
                if (ff_check_interrupt(c->interrupt_callback))
                    return AVERROR_EXIT;
                av_usleep(100*1000);
            }
            /* Enough time has elapsed since the last reload */
            goto reload;
        }

        v->input_read_done = 0;
        seg = current_segment(v);

        /* load/update Media Initialization Section, if any */
        ret = update_init_section(v, seg);
        if (ret)
            return ret;

        if (c->http_multiple == 1 && v->input_next_requested) {
            FFSWAP(AVIOContext *, v->input, v->input_next);
            v->cur_seg_offset = 0;
            v->input_next_requested = 0;
            ret = 0;
        } else {
            ret = open_input(c, v, seg, &v->input);
        }
        if (ret < 0) {
            if (ff_check_interrupt(c->interrupt_callback))
                return AVERROR_EXIT;
            av_log(v->parent, AV_LOG_WARNING, "Failed to open segment %"PRId64" of playlist %d\n",
                   v->cur_seq_no,
                   v->index);
            if (segment_retries >= c->seg_max_retry) {
                av_log(v->parent, AV_LOG_WARNING, "Segment %"PRId64" of playlist %d failed too many times, skipping\n",
                       v->cur_seq_no,
                       v->index);
                v->cur_seq_no++;
                segment_retries = 0;
            } else {
                segment_retries++;
            }
            goto reload;
        }
        segment_retries = 0;
        just_opened = 1;
    }

    if (c->http_multiple == -1) {
        uint8_t *http_version_opt = NULL;
        int r = av_opt_get(v->input, "http_version", AV_OPT_SEARCH_CHILDREN, &http_version_opt);
        if (r >= 0) {
            c->http_multiple = (!strncmp((const char *)http_version_opt, "1.1", 3) || !strncmp((const char *)http_version_opt, "2.0", 3));
            av_freep(&http_version_opt);
        }
    }

    seg = next_segment(v);
    if (c->http_multiple == 1 && !v->input_next_requested &&
        seg && seg->key_type == KEY_NONE && av_strstart(seg->url, "http", NULL)) {
        ret = open_input(c, v, seg, &v->input_next);
        if (ret < 0) {
            if (ff_check_interrupt(c->interrupt_callback))
                return AVERROR_EXIT;
            av_log(v->parent, AV_LOG_WARNING, "Failed to open segment %"PRId64" of playlist %d\n",
                   v->cur_seq_no + 1,
                   v->index);
        } else {
            v->input_next_requested = 1;
        }
    }

    if (v->init_sec_buf_read_offset < v->init_sec_data_len) {
        /* Push init section out first before first actual segment */
        int copy_size = FFMIN(v->init_sec_data_len - v->init_sec_buf_read_offset, buf_size);
        memcpy(buf, v->init_sec_buf, copy_size);
        v->init_sec_buf_read_offset += copy_size;
        return copy_size;
    }

    seg = current_segment(v);
    ret = read_from_url(v, seg, buf, buf_size);
    if (ret > 0) {
        if (just_opened && v->is_id3_timestamped != 0) {
            /* Intercept ID3 tags here, elementary audio streams are required
             * to convey timestamps using them in the beginning of each segment. */
            intercept_id3(v, buf, buf_size, &ret);
        }

        return ret;
    }
    if (c->http_persistent &&
        seg->key_type == KEY_NONE && av_strstart(seg->url, "http", NULL)) {
        v->input_read_done = 1;
    } else {
        ff_format_io_close(v->parent, &v->input);
    }
    v->cur_seq_no++;

    c->cur_seq_no = v->cur_seq_no;

    goto restart;
}

static void add_renditions_to_variant(HLSContext *c, struct variant *var,
                                      enum AVMediaType type, const char *group_id)
{
    int i;

    for (i = 0; i < c->n_renditions; i++) {
        struct rendition *rend = c->renditions[i];

        if (rend->type == type && !strcmp(rend->group_id, group_id)) {

            if (rend->playlist)
                /* rendition is an external playlist
                 * => add the playlist to the variant */
                dynarray_add(&var->playlists, &var->n_playlists, rend->playlist);
            else
                /* rendition is part of the variant main Media Playlist
                 * => add the rendition to the main Media Playlist */
                dynarray_add(&var->playlists[0]->renditions,
                             &var->playlists[0]->n_renditions,
                             rend);
        }
    }
}

static void add_metadata_from_renditions(AVFormatContext *s, struct playlist *pls,
                                         enum AVMediaType type)
{
    int rend_idx = 0;
    int i;

    for (i = 0; i < pls->n_main_streams; i++) {
        AVStream *st = pls->main_streams[i];

        if (st->codecpar->codec_type != type)
            continue;

        for (; rend_idx < pls->n_renditions; rend_idx++) {
            struct rendition *rend = pls->renditions[rend_idx];

            if (rend->type != type)
                continue;

            if (rend->language[0])
                av_dict_set(&st->metadata, "language", rend->language, 0);
            if (rend->name[0])
                av_dict_set(&st->metadata, "comment", rend->name, 0);

            st->disposition |= rend->disposition;
        }
        if (rend_idx >=pls->n_renditions)
            break;
    }
}

/* if timestamp was in valid range: returns 1 and sets seq_no
 * if not: returns 0 and sets seq_no to closest segment */
static int find_timestamp_in_playlist(HLSContext *c, struct playlist *pls,
                                      int64_t timestamp, int64_t *seq_no,
                                      int64_t *seg_start_ts)
{
    int i;
    int64_t pos = c->first_timestamp == AV_NOPTS_VALUE ?
                  0 : c->first_timestamp;

    if (timestamp < pos) {
        *seq_no = pls->start_seq_no;
        return 0;
    }

    for (i = 0; i < pls->n_segments; i++) {
        int64_t diff = pos + pls->segments[i]->duration - timestamp;
        if (diff > 0) {
            *seq_no = pls->start_seq_no + i;
            if (seg_start_ts) {
                *seg_start_ts = pos;
            }
            return 1;
        }
        pos += pls->segments[i]->duration;
    }

    *seq_no = pls->start_seq_no + pls->n_segments - 1;

    return 0;
}

static int64_t select_cur_seq_no(HLSContext *c, struct playlist *pls)
{
    int64_t seq_no;

    if (!pls->finished && !c->first_packet &&
        av_gettime_relative() - pls->last_load_time >= default_reload_interval(pls))
        /* reload the playlist since it was suspended */
        parse_playlist(c, pls->url, pls, NULL);

    /* If playback is already in progress (we are just selecting a new
     * playlist) and this is a complete file, find the matching segment
     * by counting durations. */
    if (pls->finished && c->cur_timestamp != AV_NOPTS_VALUE) {
        find_timestamp_in_playlist(c, pls, c->cur_timestamp, &seq_no, NULL);
        return seq_no;
    }

    if (!pls->finished) {
        if (!c->first_packet && /* we are doing a segment selection during playback */
            c->cur_seq_no >= pls->start_seq_no &&
            c->cur_seq_no < pls->start_seq_no + pls->n_segments)
            /* While spec 3.4.3 says that we cannot assume anything about the
             * content at the same sequence number on different playlists,
             * in practice this seems to work and doing it otherwise would
             * require us to download a segment to inspect its timestamps. */
            return c->cur_seq_no;

        /* If this is a live stream, start live_start_index segments from the
         * start or end */
        if (c->live_start_index < 0)
            seq_no = pls->start_seq_no + FFMAX(pls->n_segments +
                                            c->live_start_index, 0);
        else
            seq_no = pls->start_seq_no + FFMIN(c->live_start_index,
                                            pls->n_segments - 1);

        /* If #EXT-X-START in playlist, need to recalculate */
        if (pls->time_offset_flag && c->prefer_x_start) {
            int64_t start_timestamp;
            int64_t playlist_duration = 0;
            int64_t cur_timestamp = c->cur_timestamp == AV_NOPTS_VALUE ? 0 :
                                    c->cur_timestamp;

            for (int i = 0; i < pls->n_segments; i++)
                playlist_duration += pls->segments[i]->duration;

            /* If the absolute value of TIME-OFFSET exceeds
             * the duration of the playlist, it indicates either the end of the
             * playlist (if positive) or the beginning of the playlist (if
             * negative). */
            if (pls->start_time_offset >=0 &&
                pls->start_time_offset > playlist_duration)
                start_timestamp = cur_timestamp + playlist_duration;
            else if (pls->start_time_offset >= 0 &&
                        pls->start_time_offset <= playlist_duration)
                start_timestamp = cur_timestamp + pls->start_time_offset;
            else if (pls->start_time_offset < 0 &&
                        pls->start_time_offset < -playlist_duration)
                start_timestamp = cur_timestamp;
            else if (pls->start_time_offset < 0 &&
                        pls->start_time_offset > -playlist_duration)
                start_timestamp = cur_timestamp + playlist_duration +
                                    pls->start_time_offset;
            else
                start_timestamp = cur_timestamp;

            find_timestamp_in_playlist(c, pls, start_timestamp, &seq_no, NULL);
        }
        return seq_no;
    }

    /* Otherwise just start on the first segment. */
    return pls->start_seq_no;
}

static int nested_io_open(AVFormatContext *s, AVIOContext **pb, const char *url,
                          int flags, AVDictionary **opts)
{
    av_log(s, AV_LOG_ERROR,
           "A HLS playlist item '%s' referred to an external file '%s'. "
           "Opening this file was forbidden for security reasons\n",
           s->url, url);
    return AVERROR(EPERM);
}

static void add_stream_to_programs(AVFormatContext *s, struct playlist *pls, AVStream *stream)
{
    HLSContext *c = s->priv_data;
    int i, j;
    int bandwidth = -1;

    for (i = 0; i < c->n_variants; i++) {
        struct variant *v = c->variants[i];

        for (j = 0; j < v->n_playlists; j++) {
            if (v->playlists[j] != pls)
                continue;

            av_program_add_stream_index(s, i, stream->index);

            if (bandwidth < 0)
                bandwidth = v->bandwidth;
            else if (bandwidth != v->bandwidth)
                bandwidth = -1; /* stream in multiple variants with different bandwidths */
        }
    }

    if (bandwidth >= 0)
        av_dict_set_int(&stream->metadata, "variant_bitrate", bandwidth, 0);
}

static int set_stream_info_from_input_stream(AVStream *st, struct playlist *pls, AVStream *ist)
{
    int err;

    err = avcodec_parameters_copy(st->codecpar, ist->codecpar);
    if (err < 0)
        return err;

    if (pls->is_id3_timestamped) /* custom timestamps via id3 */
        avpriv_set_pts_info(st, 33, 1, MPEG_TIME_BASE);
    else
        avpriv_set_pts_info(st, ist->pts_wrap_bits, ist->time_base.num, ist->time_base.den);

    // copy disposition
    st->disposition = ist->disposition;

    av_dict_copy(&st->metadata, ist->metadata, 0);

    ffstream(st)->need_context_update = 1;

    return 0;
}

/* add new subdemuxer streams to our context, if any */
static int update_streams_from_subdemuxer(AVFormatContext *s, struct playlist *pls)
{
    int err;

    while (pls->n_main_streams < pls->ctx->nb_streams) {
        int ist_idx = pls->n_main_streams;
        AVStream *st = avformat_new_stream(s, NULL);
        AVStream *ist = pls->ctx->streams[ist_idx];

        if (!st)
            return AVERROR(ENOMEM);

        st->id = pls->index;
        dynarray_add(&pls->main_streams, &pls->n_main_streams, st);

        add_stream_to_programs(s, pls, st);

        err = set_stream_info_from_input_stream(st, pls, ist);
        if (err < 0)
            return err;
    }

    return 0;
}

static void update_noheader_flag(AVFormatContext *s)
{
    HLSContext *c = s->priv_data;
    int flag_needed = 0;
    int i;

    for (i = 0; i < c->n_playlists; i++) {
        struct playlist *pls = c->playlists[i];

        if (pls->has_noheader_flag) {
            flag_needed = 1;
            break;
        }
    }

    if (flag_needed)
        s->ctx_flags |= AVFMTCTX_NOHEADER;
    else
        s->ctx_flags &= ~AVFMTCTX_NOHEADER;
}

static int hls_close(AVFormatContext *s)
{
    HLSContext *c = s->priv_data;

    free_playlist_list(c);
    free_variant_list(c);
    free_rendition_list(c);

    if (c->crypto_ctx.aes_ctx)
        av_free(c->crypto_ctx.aes_ctx);

    av_dict_free(&c->avio_opts);
    ff_format_io_close(c->ctx, &c->playlist_pb);

    return 0;
}

static int hls_read_header(AVFormatContext *s)
{
    HLSContext *c = s->priv_data;
    int ret = 0, i;
    int64_t highest_cur_seq_no = 0;

    c->ctx                = s;
    c->interrupt_callback = &s->interrupt_callback;

    c->first_packet = 1;
    c->first_timestamp = AV_NOPTS_VALUE;
    c->cur_timestamp = AV_NOPTS_VALUE;

    if ((ret = ffio_copy_url_options(s->pb, &c->avio_opts)) < 0)
        return ret;

    /* XXX: Some HLS servers don't like being sent the range header,
       in this case, need to  setting http_seekable = 0 to disable
       the range header */
    av_dict_set_int(&c->avio_opts, "seekable", c->http_seekable, 0);

    if ((ret = parse_playlist(c, s->url, NULL, s->pb)) < 0)
        return ret;

    if (c->n_variants == 0) {
        av_log(s, AV_LOG_WARNING, "Empty playlist\n");
        return AVERROR_EOF;
    }
    /* If the playlist only contained playlists (Master Playlist),
     * parse each individual playlist. */
    if (c->n_playlists > 1 || c->playlists[0]->n_segments == 0) {
        for (i = 0; i < c->n_playlists; i++) {
            struct playlist *pls = c->playlists[i];
            pls->m3u8_hold_counters = 0;
            if ((ret = parse_playlist(c, pls->url, pls, NULL)) < 0) {
                av_log(s, AV_LOG_WARNING, "parse_playlist error %s [%s]\n", av_err2str(ret), pls->url);
                pls->broken = 1;
                if (c->n_playlists > 1)
                    continue;
                return ret;
            }
        }
    }

    for (i = 0; i < c->n_variants; i++) {
        if (c->variants[i]->playlists[0]->n_segments == 0) {
            av_log(s, AV_LOG_WARNING, "Empty segment [%s]\n", c->variants[i]->playlists[0]->url);
            c->variants[i]->playlists[0]->broken = 1;
        }
    }

    /* If this isn't a live stream, calculate the total duration of the
     * stream. */
    if (c->variants[0]->playlists[0]->finished) {
        int64_t duration = 0;
        for (i = 0; i < c->variants[0]->playlists[0]->n_segments; i++)
            duration += c->variants[0]->playlists[0]->segments[i]->duration;
        s->duration = duration;
    }

    /* Associate renditions with variants */
    for (i = 0; i < c->n_variants; i++) {
        struct variant *var = c->variants[i];

        if (var->audio_group[0])
            add_renditions_to_variant(c, var, AVMEDIA_TYPE_AUDIO, var->audio_group);
        if (var->video_group[0])
            add_renditions_to_variant(c, var, AVMEDIA_TYPE_VIDEO, var->video_group);
        if (var->subtitles_group[0])
            add_renditions_to_variant(c, var, AVMEDIA_TYPE_SUBTITLE, var->subtitles_group);
    }

    /* Create a program for each variant */
    for (i = 0; i < c->n_variants; i++) {
        struct variant *v = c->variants[i];
        AVProgram *program;

        program = av_new_program(s, i);
        if (!program)
            return AVERROR(ENOMEM);
        av_dict_set_int(&program->metadata, "variant_bitrate", v->bandwidth, 0);
    }

    /* Select the starting segments */
    for (i = 0; i < c->n_playlists; i++) {
        struct playlist *pls = c->playlists[i];

        if (pls->n_segments == 0)
            continue;

        pls->cur_seq_no = select_cur_seq_no(c, pls);
        highest_cur_seq_no = FFMAX(highest_cur_seq_no, pls->cur_seq_no);
    }

    /* Open the demuxer for each playlist */
    for (i = 0; i < c->n_playlists; i++) {
        struct playlist *pls = c->playlists[i];
        const AVInputFormat *in_fmt = NULL;
        char *url;
        AVDictionary *options = NULL;
        struct segment *seg = NULL;

        if (!(pls->ctx = avformat_alloc_context()))
            return AVERROR(ENOMEM);

        if (pls->n_segments == 0)
            continue;

        pls->index  = i;
        pls->needed = 1;
        pls->parent = s;

        /*
         * If this is a live stream and this playlist looks like it is one segment
         * behind, try to sync it up so that every substream starts at the same
         * time position (so e.g. avformat_find_stream_info() will see packets from
         * all active streams within the first few seconds). This is not very generic,
         * though, as the sequence numbers are technically independent.
         */
        if (!pls->finished && pls->cur_seq_no == highest_cur_seq_no - 1 &&
            highest_cur_seq_no < pls->start_seq_no + pls->n_segments) {
            pls->cur_seq_no = highest_cur_seq_no;
        }

        pls->read_buffer = av_malloc(INITIAL_BUFFER_SIZE);
        if (!pls->read_buffer){
            avformat_free_context(pls->ctx);
            pls->ctx = NULL;
            return AVERROR(ENOMEM);
        }

        ffio_init_context(&pls->pb, pls->read_buffer, INITIAL_BUFFER_SIZE, 0, pls,
                          read_data, NULL, NULL);

        /*
         * If encryption scheme is SAMPLE-AES, try to read  ID3 tags of
         * external audio track that contains audio setup information
         */
        seg = current_segment(pls);
        if (seg && seg->key_type == KEY_SAMPLE_AES && pls->n_renditions > 0 &&
            pls->renditions[0]->type == AVMEDIA_TYPE_AUDIO) {
            uint8_t buf[HLS_MAX_ID3_TAGS_DATA_LEN];
            if ((ret = avio_read(&pls->pb.pub, buf, HLS_MAX_ID3_TAGS_DATA_LEN)) < 0) {
                /* Fail if error was not end of file */
                if (ret != AVERROR_EOF) {
                    avformat_free_context(pls->ctx);
                    pls->ctx = NULL;
                    return ret;
                }
            }
            ret = 0;
            /* Reset reading */
            ff_format_io_close(pls->parent, &pls->input);
            pls->input = NULL;
            pls->input_read_done = 0;
            ff_format_io_close(pls->parent, &pls->input_next);
            pls->input_next = NULL;
            pls->input_next_requested = 0;
            pls->cur_seg_offset = 0;
            pls->cur_init_section = NULL;
            /* Reset EOF flag */
            pls->pb.pub.eof_reached = 0;
            /* Clear any buffered data */
            pls->pb.pub.buf_end = pls->pb.pub.buf_ptr = pls->pb.pub.buffer;
            /* Reset the position */
            pls->pb.pub.pos = 0;
        }

        /*
         * If encryption scheme is SAMPLE-AES and audio setup information is present in external audio track,
         * use that information to find the media format, otherwise probe input data
         */
        seg = current_segment(pls);
        if (seg && seg->key_type == KEY_SAMPLE_AES && pls->is_id3_timestamped &&
            pls->audio_setup_info.codec_id != AV_CODEC_ID_NONE) {
            av_assert1(pls->audio_setup_info.codec_id == AV_CODEC_ID_AAC ||
                       pls->audio_setup_info.codec_id == AV_CODEC_ID_AC3 ||
                       pls->audio_setup_info.codec_id == AV_CODEC_ID_EAC3);
            // Keep this list in sync with ff_hls_senc_read_audio_setup_info()
            in_fmt = av_find_input_format(pls->audio_setup_info.codec_id == AV_CODEC_ID_AAC ? "aac" :
                                          pls->audio_setup_info.codec_id == AV_CODEC_ID_AC3 ? "ac3" : "eac3");
        } else {
            pls->ctx->probesize = s->probesize > 0 ? s->probesize : 1024 * 4;
            pls->ctx->max_analyze_duration = s->max_analyze_duration > 0 ? s->max_analyze_duration : 4 * AV_TIME_BASE;
            pls->ctx->interrupt_callback = s->interrupt_callback;
            url = av_strdup(pls->segments[0]->url);
            ret = av_probe_input_buffer(&pls->pb.pub, &in_fmt, url, NULL, 0, 0);

            for (int n = 0; n < pls->n_segments; n++)
                if (ret >= 0)
                    ret = test_segment(s, in_fmt, pls, pls->segments[n]);

            if (ret < 0) {
                /* Free the ctx - it isn't initialized properly at this point,
                * so avformat_close_input shouldn't be called. If
                * avformat_open_input fails below, it frees and zeros the
                * context, so it doesn't need any special treatment like this. */
                av_log(s, AV_LOG_ERROR, "Error when loading first segment '%s'\n", url);
                avformat_free_context(pls->ctx);
                pls->ctx = NULL;
                av_free(url);
                return ret;
            }
            av_free(url);
        }

        seg = current_segment(pls);
        if (seg && seg->key_type == KEY_SAMPLE_AES) {
            if (strstr(in_fmt->name, "mov")) {
                char key[33];
                ff_data_to_hex(key, pls->key, sizeof(pls->key), 0);
                av_dict_set(&options, "decryption_key", key, 0);
            } else if (!c->crypto_ctx.aes_ctx) {
                c->crypto_ctx.aes_ctx = av_aes_alloc();
                if (!c->crypto_ctx.aes_ctx) {
                    avformat_free_context(pls->ctx);
                    pls->ctx = NULL;
                    return AVERROR(ENOMEM);
                }
            }
        }

        pls->ctx->pb       = &pls->pb.pub;
        pls->ctx->io_open  = nested_io_open;
        pls->ctx->flags   |= s->flags & ~AVFMT_FLAG_CUSTOM_IO;

        if ((ret = ff_copy_whiteblacklists(pls->ctx, s)) < 0)
            return ret;

        av_dict_copy(&options, c->seg_format_opts, 0);

        ret = avformat_open_input(&pls->ctx, pls->segments[0]->url, in_fmt, &options);
        av_dict_free(&options);
        if (ret < 0)
            return ret;

        if (pls->id3_deferred_extra && pls->ctx->nb_streams == 1) {
            ff_id3v2_parse_apic(pls->ctx, pls->id3_deferred_extra);
            avformat_queue_attached_pictures(pls->ctx);
            ff_id3v2_parse_priv(pls->ctx, pls->id3_deferred_extra);
            ff_id3v2_free_extra_meta(&pls->id3_deferred_extra);
        }

        if (pls->is_id3_timestamped == -1)
            av_log(s, AV_LOG_WARNING, "No expected HTTP requests have been made\n");

        /*
         * For ID3 timestamped raw audio streams we need to detect the packet
         * durations to calculate timestamps in fill_timing_for_id3_timestamped_stream(),
         * but for other streams we can rely on our user calling avformat_find_stream_info()
         * on us if they want to.
         */
        if (pls->is_id3_timestamped || (pls->n_renditions > 0 && pls->renditions[0]->type == AVMEDIA_TYPE_AUDIO)) {
            seg = current_segment(pls);
            if (seg && seg->key_type == KEY_SAMPLE_AES && pls->audio_setup_info.setup_data_length > 0 &&
                pls->ctx->nb_streams == 1)
                ret = ff_hls_senc_parse_audio_setup_info(pls->ctx->streams[0], &pls->audio_setup_info);
            else
                ret = avformat_find_stream_info(pls->ctx, NULL);

            if (ret < 0)
                return ret;
        }

        pls->has_noheader_flag = !!(pls->ctx->ctx_flags & AVFMTCTX_NOHEADER);

        /* Create new AVStreams for each stream in this playlist */
        ret = update_streams_from_subdemuxer(s, pls);
        if (ret < 0)
            return ret;

        /*
         * Copy any metadata from playlist to main streams, but do not set
         * event flags.
         */
        if (pls->n_main_streams)
            av_dict_copy(&pls->main_streams[0]->metadata, pls->ctx->metadata, 0);

        add_metadata_from_renditions(s, pls, AVMEDIA_TYPE_AUDIO);
        add_metadata_from_renditions(s, pls, AVMEDIA_TYPE_VIDEO);
        add_metadata_from_renditions(s, pls, AVMEDIA_TYPE_SUBTITLE);
    }

    update_noheader_flag(s);

    return 0;
}

static int recheck_discard_flags(AVFormatContext *s, int first)
{
    HLSContext *c = s->priv_data;
    int i, changed = 0;
    int cur_needed;

    /* Check if any new streams are needed */
    for (i = 0; i < c->n_playlists; i++) {
        struct playlist *pls = c->playlists[i];

        cur_needed = playlist_needed(c->playlists[i]);

        if (pls->broken) {
            continue;
        }
        if (cur_needed && !pls->needed) {
            pls->needed = 1;
            changed = 1;
            pls->cur_seq_no = select_cur_seq_no(c, pls);
            pls->pb.pub.eof_reached = 0;
            if (c->cur_timestamp != AV_NOPTS_VALUE) {
                /* catch up */
                pls->seek_timestamp = c->cur_timestamp;
                pls->seek_flags = AVSEEK_FLAG_ANY;
                pls->seek_stream_index = -1;
            }
            av_log(s, AV_LOG_INFO, "Now receiving playlist %d, segment %"PRId64"\n", i, pls->cur_seq_no);
        } else if (first && !cur_needed && pls->needed) {
            ff_format_io_close(pls->parent, &pls->input);
            pls->input_read_done = 0;
            ff_format_io_close(pls->parent, &pls->input_next);
            pls->input_next_requested = 0;
            pls->needed = 0;
            changed = 1;
            av_log(s, AV_LOG_INFO, "No longer receiving playlist %d\n", i);
        }
    }
    return changed;
}

static void fill_timing_for_id3_timestamped_stream(struct playlist *pls)
{
    if (pls->id3_offset >= 0) {
        pls->pkt->dts = pls->id3_mpegts_timestamp +
                                 av_rescale_q(pls->id3_offset,
                                              pls->ctx->streams[pls->pkt->stream_index]->time_base,
                                              MPEG_TIME_BASE_Q);
        if (pls->pkt->duration)
            pls->id3_offset += pls->pkt->duration;
        else
            pls->id3_offset = -1;
    } else {
        /* there have been packets with unknown duration
         * since the last id3 tag, should not normally happen */
        pls->pkt->dts = AV_NOPTS_VALUE;
    }

    if (pls->pkt->duration)
        pls->pkt->duration = av_rescale_q(pls->pkt->duration,
                                         pls->ctx->streams[pls->pkt->stream_index]->time_base,
                                         MPEG_TIME_BASE_Q);

    pls->pkt->pts = AV_NOPTS_VALUE;
}

static AVRational get_timebase(struct playlist *pls)
{
    if (pls->is_id3_timestamped)
        return MPEG_TIME_BASE_Q;

    return pls->ctx->streams[pls->pkt->stream_index]->time_base;
}

static int compare_ts_with_wrapdetect(int64_t ts_a, struct playlist *pls_a,
                                      int64_t ts_b, struct playlist *pls_b)
{
    int64_t scaled_ts_a = av_rescale_q(ts_a, get_timebase(pls_a), MPEG_TIME_BASE_Q);
    int64_t scaled_ts_b = av_rescale_q(ts_b, get_timebase(pls_b), MPEG_TIME_BASE_Q);

    return av_compare_mod(scaled_ts_a, scaled_ts_b, 1LL << 33);
}

static int hls_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    HLSContext *c = s->priv_data;
    int ret, i, minplaylist = -1;

    recheck_discard_flags(s, c->first_packet);
    c->first_packet = 0;

    for (i = 0; i < c->n_playlists; i++) {
        struct playlist *pls = c->playlists[i];
        /* Make sure we've got one buffered packet from each open playlist
         * stream */
        if (pls->needed && !pls->pkt->data) {
            while (1) {
                int64_t ts_diff;
                AVRational tb;
                struct segment *seg = NULL;
                ret = av_read_frame(pls->ctx, pls->pkt);
                if (ret < 0) {
                    if (!avio_feof(&pls->pb.pub) && ret != AVERROR_EOF)
                        return ret;
                    break;
                } else {
                    /* stream_index check prevents matching picture attachments etc. */
                    if (pls->is_id3_timestamped && pls->pkt->stream_index == 0) {
                        /* audio elementary streams are id3 timestamped */
                        fill_timing_for_id3_timestamped_stream(pls);
                    }

                    if (c->first_timestamp == AV_NOPTS_VALUE &&
                        pls->pkt->dts       != AV_NOPTS_VALUE)
                        c->first_timestamp = av_rescale_q(pls->pkt->dts,
                            get_timebase(pls), AV_TIME_BASE_Q);
                }

                seg = current_segment(pls);
                if (seg && seg->key_type == KEY_SAMPLE_AES && !strstr(pls->ctx->iformat->name, "mov")) {
                    enum AVCodecID codec_id = pls->ctx->streams[pls->pkt->stream_index]->codecpar->codec_id;
                    memcpy(c->crypto_ctx.iv, seg->iv, sizeof(seg->iv));
                    memcpy(c->crypto_ctx.key, pls->key, sizeof(pls->key));
                    ff_hls_senc_decrypt_frame(codec_id, &c->crypto_ctx, pls->pkt);
                }

                if (pls->seek_timestamp == AV_NOPTS_VALUE)
                    break;

                if (pls->seek_stream_index < 0 ||
                    pls->seek_stream_index == pls->pkt->stream_index) {

                    if (pls->pkt->dts == AV_NOPTS_VALUE) {
                        pls->seek_timestamp = AV_NOPTS_VALUE;
                        break;
                    }

                    tb = get_timebase(pls);
                    ts_diff = av_rescale_rnd(pls->pkt->dts, AV_TIME_BASE,
                                            tb.den, AV_ROUND_DOWN) -
                            pls->seek_timestamp;
                    if (ts_diff >= 0 && (pls->seek_flags  & AVSEEK_FLAG_ANY ||
                                        pls->pkt->flags & AV_PKT_FLAG_KEY)) {
                        pls->seek_timestamp = AV_NOPTS_VALUE;
                        break;
                    }
                }
                av_packet_unref(pls->pkt);
            }
        }
        /* Check if this stream has the packet with the lowest dts */
        if (pls->pkt->data) {
            struct playlist *minpls = minplaylist < 0 ?
                                     NULL : c->playlists[minplaylist];
            if (minplaylist < 0) {
                minplaylist = i;
            } else {
                int64_t dts     =    pls->pkt->dts;
                int64_t mindts  = minpls->pkt->dts;

                if (dts == AV_NOPTS_VALUE ||
                    (mindts != AV_NOPTS_VALUE && compare_ts_with_wrapdetect(dts, pls, mindts, minpls) < 0))
                    minplaylist = i;
            }
        }
    }

    /* If we got a packet, return it */
    if (minplaylist >= 0) {
        struct playlist *pls = c->playlists[minplaylist];
        AVStream *ist;
        AVStream *st;

        ret = update_streams_from_subdemuxer(s, pls);
        if (ret < 0) {
            av_packet_unref(pls->pkt);
            return ret;
        }

        // If sub-demuxer reports updated metadata, copy it to the first stream
        // and set its AVSTREAM_EVENT_FLAG_METADATA_UPDATED flag.
        if (pls->ctx->event_flags & AVFMT_EVENT_FLAG_METADATA_UPDATED) {
            if (pls->n_main_streams) {
                st = pls->main_streams[0];
                av_dict_copy(&st->metadata, pls->ctx->metadata, 0);
                st->event_flags |= AVSTREAM_EVENT_FLAG_METADATA_UPDATED;
            }
            pls->ctx->event_flags &= ~AVFMT_EVENT_FLAG_METADATA_UPDATED;
        }

        /* check if noheader flag has been cleared by the subdemuxer */
        if (pls->has_noheader_flag && !(pls->ctx->ctx_flags & AVFMTCTX_NOHEADER)) {
            pls->has_noheader_flag = 0;
            update_noheader_flag(s);
        }

        if (pls->pkt->stream_index >= pls->n_main_streams) {
            av_log(s, AV_LOG_ERROR, "stream index inconsistency: index %d, %d main streams, %d subdemuxer streams\n",
                   pls->pkt->stream_index, pls->n_main_streams, pls->ctx->nb_streams);
            av_packet_unref(pls->pkt);
            return AVERROR_BUG;
        }

        ist = pls->ctx->streams[pls->pkt->stream_index];
        st = pls->main_streams[pls->pkt->stream_index];

        av_packet_move_ref(pkt, pls->pkt);
        pkt->stream_index = st->index;

        if (pkt->dts != AV_NOPTS_VALUE)
            c->cur_timestamp = av_rescale_q(pkt->dts,
                                            ist->time_base,
                                            AV_TIME_BASE_Q);

        /* There may be more situations where this would be useful, but this at least
         * handles newly probed codecs properly (i.e. request_probe by mpegts). */
        if (ist->codecpar->codec_id != st->codecpar->codec_id) {
            ret = set_stream_info_from_input_stream(st, pls, ist);
            if (ret < 0) {
                return ret;
            }
        }

        return 0;
    }
    return AVERROR_EOF;
}

static int hls_read_seek(AVFormatContext *s, int stream_index,
                               int64_t timestamp, int flags)
{
    HLSContext *c = s->priv_data;
    struct playlist *seek_pls = NULL;
    int i, j;
    int stream_subdemuxer_index;
    int64_t first_timestamp, seek_timestamp, duration;
    int64_t seq_no, seg_start_ts;

    if ((flags & AVSEEK_FLAG_BYTE) || (c->ctx->ctx_flags & AVFMTCTX_UNSEEKABLE))
        return AVERROR(ENOSYS);

    first_timestamp = c->first_timestamp == AV_NOPTS_VALUE ?
                      0 : c->first_timestamp;

    seek_timestamp = av_rescale_rnd(timestamp, AV_TIME_BASE,
                                    s->streams[stream_index]->time_base.den,
                                    AV_ROUND_DOWN);

    duration = s->duration == AV_NOPTS_VALUE ?
               0 : s->duration;

    if (0 < duration && duration < seek_timestamp - first_timestamp)
        return AVERROR(EIO);

    /* find the playlist with the specified stream */
    for (i = 0; i < c->n_playlists; i++) {
        struct playlist *pls = c->playlists[i];
        for (j = 0; j < pls->n_main_streams; j++) {
            if (pls->main_streams[j] == s->streams[stream_index]) {
                seek_pls = pls;
                stream_subdemuxer_index = j;
                break;
            }
        }
    }
    /* check if the timestamp is valid for the playlist with the
     * specified stream index */
    if (!seek_pls || !find_timestamp_in_playlist(c, seek_pls, seek_timestamp, &seq_no, &seg_start_ts))
        return AVERROR(EIO);

    if (s->streams[stream_index]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO &&
        flags & AVSEEK_FLAG_BACKWARD && !(flags & AVSEEK_FLAG_ANY)) {
        /* Seeking to start of segment ensures we seek to a keyframe located
         * before the given timestamp. */
        seek_timestamp = seg_start_ts;
    }

    /* set segment now so we do not need to search again below */
    seek_pls->cur_seq_no = seq_no;
    seek_pls->seek_stream_index = stream_subdemuxer_index;

    for (i = 0; i < c->n_playlists; i++) {
        /* Reset reading */
        struct playlist *pls = c->playlists[i];
        AVIOContext *const pb = &pls->pb.pub;
        ff_format_io_close(pls->parent, &pls->input);
        pls->input_read_done = 0;
        ff_format_io_close(pls->parent, &pls->input_next);
        pls->input_next_requested = 0;
        av_packet_unref(pls->pkt);
        pb->eof_reached = 0;
        /* Clear any buffered data */
        pb->buf_end = pb->buf_ptr = pb->buffer;
        /* Reset the pos, to let the mpegts demuxer know we've seeked. */
        pb->pos = 0;
        /* Flush the packet queue of the subdemuxer. */
        ff_read_frame_flush(pls->ctx);

        /* Reset the init segment so it's re-fetched and served appropiately */
        pls->cur_init_section = NULL;

        pls->seek_timestamp = seek_timestamp;
        pls->seek_flags = flags;

        if (pls != seek_pls) {
            /* set closest segment seq_no for playlists not handled above */
            find_timestamp_in_playlist(c, pls, seek_timestamp, &pls->cur_seq_no, NULL);
            /* seek the playlist to the given position without taking
             * keyframes into account since this playlist does not have the
             * specified stream where we should look for the keyframes */
            pls->seek_stream_index = -1;
            pls->seek_flags |= AVSEEK_FLAG_ANY;
        }
    }

    c->cur_timestamp = seek_timestamp;

    return 0;
}

static int hls_probe(const AVProbeData *p)
{
    /* Require #EXTM3U at the start, and either one of the ones below
     * somewhere for a proper match. */
    if (strncmp(p->buf, "#EXTM3U", 7))
        return 0;

    if (strstr(p->buf, "#EXT-X-STREAM-INF:")     ||
        strstr(p->buf, "#EXT-X-TARGETDURATION:") ||
        strstr(p->buf, "#EXT-X-MEDIA-SEQUENCE:")) {

        int mime_ok = p->mime_type && !(
            av_strcasecmp(p->mime_type, "application/vnd.apple.mpegurl") &&
            av_strcasecmp(p->mime_type, "audio/mpegurl")
            );

        int mime_x = p->mime_type && !(
            av_strcasecmp(p->mime_type, "audio/x-mpegurl") &&
            av_strcasecmp(p->mime_type, "application/x-mpegurl")
            );

        if (!mime_ok &&
            !mime_x &&
            !av_match_ext    (p->filename, "m3u8,m3u") &&
             ff_match_url_ext(p->filename, "m3u8,m3u") <= 0) {
            av_log(NULL, AV_LOG_ERROR, "Not detecting m3u8/hls with non standard extension and non standard mime type\n");
            return 0;
        }
        if (mime_x)
            av_log(NULL, AV_LOG_WARNING, "mime type is not rfc8216 compliant\n");

        return AVPROBE_SCORE_MAX;
    }
    return 0;
}

#define OFFSET(x) offsetof(HLSContext, x)
#define FLAGS AV_OPT_FLAG_DECODING_PARAM
static const AVOption hls_options[] = {
    {"live_start_index", "segment index to start live streams at (negative values are from the end)",
        OFFSET(live_start_index), AV_OPT_TYPE_INT, {.i64 = -3}, INT_MIN, INT_MAX, FLAGS},
    {"prefer_x_start", "prefer to use #EXT-X-START if it's in playlist instead of live_start_index",
        OFFSET(prefer_x_start), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, FLAGS},
    {"allowed_extensions", "List of file extensions that hls is allowed to access",
        OFFSET(allowed_extensions), AV_OPT_TYPE_STRING,
        {.str = "3gp,aac,avi,ac3,eac3,flac,mkv,m3u8,m4a,m4s,m4v,mpg,mov,mp2,mp3,mp4,mpeg,mpegts,ogg,ogv,oga,ts,vob,vtt,wav,webvtt"
            ",cmfv,cmfa" // Ticket11526 www.nicovideo.jp
            ",ec3"       // part of Ticket11435 (Elisa Viihde (Finnish online recording service))
            ",fmp4"      // https://github.com/yt-dlp/yt-dlp/issues/12700
        },
        INT_MIN, INT_MAX, FLAGS},
    {"allowed_segment_extensions", "List of file extensions that hls is allowed to access",
        OFFSET(allowed_segment_extensions), AV_OPT_TYPE_STRING,
        {.str = "3gp,aac,avi,ac3,eac3,flac,mkv,m3u8,m4a,m4s,m4v,mpg,mov,mp2,mp3,mp4,mpeg,mpegts,ogg,ogv,oga,ts,vob,vtt,wav,webvtt"
            ",cmfv,cmfa" // Ticket11526 www.nicovideo.jp
            ",ec3"       // part of Ticket11435 (Elisa Viihde (Finnish online recording service))
            ",fmp4"      // https://github.com/yt-dlp/yt-dlp/issues/12700
            ",html"      // https://flash1.bogulus.cfd/
        },
        INT_MIN, INT_MAX, FLAGS},
    {"extension_picky", "Be picky with all extensions matching",
        OFFSET(extension_picky), AV_OPT_TYPE_BOOL, {.i64 = 1}, 0, 1, FLAGS},
    {"max_reload", "Maximum number of times a insufficient list is attempted to be reloaded",
        OFFSET(max_reload), AV_OPT_TYPE_INT, {.i64 = 100}, 0, INT_MAX, FLAGS},
    {"m3u8_hold_counters", "The maximum number of times to load m3u8 when it refreshes without new segments",
        OFFSET(m3u8_hold_counters), AV_OPT_TYPE_INT, {.i64 = 1000}, 0, INT_MAX, FLAGS},
    {"http_persistent", "Use persistent HTTP connections",
        OFFSET(http_persistent), AV_OPT_TYPE_BOOL, {.i64 = 1}, 0, 1, FLAGS },
    {"http_multiple", "Use multiple HTTP connections for fetching segments",
        OFFSET(http_multiple), AV_OPT_TYPE_BOOL, {.i64 = -1}, -1, 1, FLAGS},
    {"http_seekable", "Use HTTP partial requests, 0 = disable, 1 = enable, -1 = auto",
        OFFSET(http_seekable), AV_OPT_TYPE_BOOL, { .i64 = -1}, -1, 1, FLAGS},
    {"seg_format_options", "Set options for segment demuxer",
        OFFSET(seg_format_opts), AV_OPT_TYPE_DICT, {.str = NULL}, 0, 0, FLAGS},
    {"seg_max_retry", "Maximum number of times to reload a segment on error.",
     OFFSET(seg_max_retry), AV_OPT_TYPE_INT, {.i64 = 0}, 0, INT_MAX, FLAGS},
    {NULL}
};

static const AVClass hls_class = {
    .class_name = "hls demuxer",
    .item_name  = av_default_item_name,
    .option     = hls_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFInputFormat ff_hls_demuxer = {
    .p.name         = "hls",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Apple HTTP Live Streaming"),
    .p.priv_class   = &hls_class,
    .p.flags        = AVFMT_NOGENSEARCH | AVFMT_TS_DISCONT | AVFMT_NO_BYTE_SEEK,
    .priv_data_size = sizeof(HLSContext),
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = hls_probe,
    .read_header    = hls_read_header,
    .read_packet    = hls_read_packet,
    .read_close     = hls_close,
    .read_seek      = hls_read_seek,
};
