fn main() {
    println!("cargo:rustc-link-search=native=../fftools");
    println!("cargo:rustc-link-search=native=../libavdevice");
    println!("cargo:rustc-link-search=native=../libavfilter");
    println!("cargo:rustc-link-search=native=../libavformat");
    println!("cargo:rustc-link-search=native=../libavcodec");
    println!("cargo:rustc-link-search=native=../libswscale");
    println!("cargo:rustc-link-search=native=../libswresample");
    println!("cargo:rustc-link-search=native=../libavutil");

    println!("cargo:rustc-link-arg=-Wl,--start-group");
    println!("cargo:rustc-link-lib=static=fftools");
    println!("cargo:rustc-link-lib=static=avdevice");
    println!("cargo:rustc-link-lib=static=avfilter");
    println!("cargo:rustc-link-lib=static=avformat");
    println!("cargo:rustc-link-lib=static=avcodec");
    println!("cargo:rustc-link-lib=static=swscale");
    println!("cargo:rustc-link-lib=static=swresample");
    println!("cargo:rustc-link-lib=static=avutil");
    println!("cargo:rustc-link-arg=-Wl,--end-group");

    println!("cargo:rustc-link-lib=xcb");
    println!("cargo:rustc-link-lib=m");
    println!("cargo:rustc-link-lib=z");
    println!("cargo:rustc-link-lib=pthread");
}
