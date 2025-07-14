fn main() {
    println!("cargo:rustc-link-search=native=../libavcodec");
    println!("cargo:rustc-link-lib=avcodec");
    println!("cargo:rustc-link-search=native=../libavformat");
    println!("cargo:rustc-link-lib=avformat");
    println!("cargo:rustc-link-search=native=../libavdevice");
    println!("cargo:rustc-link-lib=avdevice");
    println!("cargo:rustc-link-search=native=../libavutil");
    println!("cargo:rustc-link-lib=avutil");
    println!("cargo:rustc-link-search=native=../libswresample");
    println!("cargo:rustc-link-lib=swresample");
    println!("cargo:rustc-link-search=native=../libswscale");
    println!("cargo:rustc-link-lib=swscale");
    println!("cargo:rustc-link-lib=m");
    println!("cargo:rustc-link-lib=z");
    println!("cargo:rustc-link-lib=pthread");
} 