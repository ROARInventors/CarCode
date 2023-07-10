{pkgs ? import <nixpkgs> {} }:
pkgs.mkShell {
	packages = with pkgs; [ 
		(python3.withPackages (ps: with ps; [
			# Base (not in nixpkgs: open3d)
			pydantic pygame numpy scipy opencv4 pillow prettytable matplotlib
			# ROAR_Sim
			imageio imageio-ffmpeg moviepy pandas pyrealsense2WithoutCuda pyserial
		]))
		lazygit
	];
}
