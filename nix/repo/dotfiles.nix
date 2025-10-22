{ inputs, cell }:
let
  inherit (inputs) nixpkgs;
  inherit (inputs.std) lib;

  l = builtins // nixpkgs.lib;
in
{

  treefmt = lib.dev.mkNixago lib.cfg.treefmt {
    packages = [
      nixpkgs.nixfmt
      nixpkgs.prettier
      nixpkgs.shellcheck
      nixpkgs.taplo
    ];
    data = {
      global = {
        on-unmatched = "warn";
        excludes = [ "CHANGELOG.md" ];
      };
      formatter = {
        nixfmt = {
          command = "nixfmt";
          includes = [ "*.nix" ];
        };
        prettier = {
          command = "prettier";
          options = [ "--write" ];
          includes = [
            "*.json"
            "*.md"
            "*.yaml"
          ];
        };
        taplo = {
          command = "taplo";
          options = [ "format" ];
          includes = [ "*.toml" ];
        };
        shellcheck = {
          command = "shellcheck";
          includes = [ "*.sh" ];
        };
      };
    };
  };

  lefthook = lib.dev.mkNixago lib.cfg.lefthook {
    packages = [
      nixpkgs.actionlint
      nixpkgs.ripsecrets
      nixpkgs.shellcheck # used by actionlint
    ];
    data = {
      pre-push = {
        parallel = true;
        commands = {
          cocogitto.run = "cog check -l";
          django-test.run = "just test";
        };
      };
      pre-commit = {
        parallel = true;
        fail_on_changes = "ci"; # only when $CI=1
        commands = {
          actionlint = {
            run = "actionlint '{staged_files}'";
            glob = ".github/workflows/*.yaml";
          };
          ripsecrets = {
            run = "ripsecrets --strict-ignore '{staged_files}'";
            file_types = [ "text" ];
          };
          treefmt = {
            run = "treefmt '{staged_files}'";
            stage_fixed = true;
          };
        };
      };
    };
  };

}
