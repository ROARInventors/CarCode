{pkgs ? import <nixpkgs> {} }:
pkgs.mkShell {
	packages = with pkgs; [ 
		(python3.withPackages (ps: with ps; [
			# Base
			pydantic pygame numpy scipy opencv4 pillow prettytable matplotlib
			# ROAR_Sim
			imageio imageio-ffmpeg moviepy pandas pyrealsense2WithoutCuda pyserial
			# TODO
			# - open3d
			# - https://drive.google.com/drive/folders/13JSejJED31cZHBbfIz_gyxxPmiqABOJj?usp=sharing
			# - https://drive.google.com/drive/folders/1ejKIOp8_vXaTroA7prcCDrfQet9WL-oD?usp=sharing
		]))
	];
}
