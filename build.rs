use std::env;
use std::fs;
use std::path::Path;
use std::process::{Command, Stdio};

fn main() {
    let dest: String = concat!("target/doc/", env!("CARGO_PKG_NAME")).to_string();

    let _ = mml::src2both("src", dest.replace("-", "_").as_str());

    let manifest_dir = env::var_os("CARGO_MANIFEST_DIR").unwrap();
    #[cfg(target_os = "linux")]
    let glslang_path = Path::new(&manifest_dir).join("tools/glslangValidator_unix");
    #[cfg(target_os = "macos")]
    let glslang_path = Path::new(&manifest_dir).join("tools/glslangValidator");
    #[cfg(target_os = "windows")]
    let glslang_path = Path::new(&manifest_dir).join("tools/glslangValidator.exe");

    let shader_dir = Path::new(&manifest_dir).join("shaders");
    let shaders = fs::read_dir(&shader_dir).unwrap();

    let mut file_path;
    let mut extension;
    for shader in shaders {
        if let Ok(shader) = shader {
            file_path = shader.path();
            extension = file_path.extension().unwrap().to_str().unwrap();
            if extension == "vert" || extension == "frag" {
                let mut glslang_cmd = Command::new(&glslang_path);
                glslang_cmd
                    .arg("-V")
                    .arg("-o")
                    .arg(&format!("{}.spv", file_path.to_str().unwrap()))
                    .arg(&file_path);

                if glslang_cmd
                    .stdout(Stdio::inherit())
                    .stderr(Stdio::inherit())
                    .status()
                    .unwrap()
                    .code()
                    .unwrap()
                    != 0
                {
                    panic!("Error while compiling spirv: {:?}", file_path);
                };
                println!("cargo:rerun-if-changed={}", file_path.to_str().unwrap());
            }
        }
    }
}
