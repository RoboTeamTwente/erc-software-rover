{
  # std
  inputs = {
    devshell.url = "github:numtide/devshell";
    n2c.url = "github:nlewo/nix2container";
    nixago.url = "github:nix-community/nixago";
    nixpkgs.url = "https://channels.nixos.org/nixos-unstable/nixexprs.tar.xz";
    std = {
      inputs.devshell.follows = "devshell";
      inputs.n2c.follows = "n2c";
      inputs.nixago.follows = "nixago";
      url = "github:divnix/std";
    };
  };

  outputs =
    inputs:
    inputs.std.growOn
      {
        inherit inputs;
        cellsFrom = ./nix;
        cellBlocks = with inputs.std.blockTypes; [
          (containers "containers")
          (devshells "shells" { ci.build = true; })
          (installables "packages" { ci.build = true; })
          (nixago "dotfiles")
          (runnables "operables" { ci.build = true; })
          (runnables "scripts" { ci.build = true; })
        ];
      }
      {
        devShells = inputs.std.harvest inputs.self [
          "repo"
          "shells"
        ];
        packages = inputs.std.harvest inputs.self [
          "repo"
          "packages"
        ];
      };
}
