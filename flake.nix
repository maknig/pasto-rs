{
  description = "Rust project for ARM (thumbv7em-none-eabi)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs = { self, nixpkgs, rust-overlay }:
    let
      system = "x86_64-linux"; # adjust if you're on aarch64
      overlays = [ rust-overlay.overlays.default ];

      pkgs = import nixpkgs { inherit system overlays; };
      rust = pkgs.rust-bin.stable.latest.default.override {
        targets = [ "thumbv7em-none-eabi" ];
      };
    in {
      devShells.${system}.default = pkgs.mkShell {
        buildInputs = [
          pkgs.probe-rs-tools
          rust
          pkgs.llvmPackages.bintools
          pkgs.qemu # optional: for emulation/testing
        ];

        shellHook = ''
          echo "Rust ARM dev shell ready!"
          rustc --version
          echo "Targets installed:"
          rustc --print target-list | grep thumbv7em
        '';
      };
    };
}

