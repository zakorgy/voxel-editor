# voxel-editor

A cross-platform simple voxel editor written is rust, using wgpu-rs.
The UI elements are composed with the iced-wgpu wrapper

![Example image](https://github.com/zakorgy/voxel-editor/blob/master/img/Astar.PNG)
![Example image](https://github.com/zakorgy/voxel-editor/blob/master/img/MacOS.PNG)

## Install dependecies

### Install Rustup
Download and install `rustup` from https://rustup.rs/

You can test it with:
```bash
rustc -V
```

### Linux dependency
Install Linux gtk dependency
```bash
sudo apt update && sudo apt install libgtk-3-dev
```

### Windows build fix
Before building, first you need to fix an issue with the following command:
```bash
cargo update -p error-code --precise 2.0.0
```

## Build and Run
You can build the project with:
```bash
cargo build --release
```

You can run the application with:
```bash
cargo run --release
```

## Development
You can build the app with "--debug" to get better backtrace.

You can run the tests with
```bash
cargo test
```
