import os
import subprocess
import plistlib
import shutil

from conan.tools.files import get, copy, load, download
from conan import ConanFile

class MujocoConan(ConanFile):
    name = "mujoco"
    settings = "os", "arch"

    def set_version(self):
        self.version = load(self, os.path.join(self.recipe_folder, "version.txt")).strip()

    def package(self):
        url_format = "https://github.com/google-deepmind/mujoco/releases/download/{version}/mujoco-{version}-{file}"

        os_setting = str(self.settings.os)
        if os_setting == "Windows":
            file = "windows-x86_64.zip"
        elif os_setting == "Macos":
            file = "macos-universal2.dmg"
        elif os_setting == "Linux":
            if str(self.settings.arch).startswith("arm"):
                file = "linux-aarch64.tar.gz"
            else:
                file = "linux-x86_64.tar.gz"
        else:
            raise ValueError(f"Unknown OS {os_setting}")

        url = url_format.format(version=self.version, file=file)

        if os_setting == "Linux" or os_setting == "Windows":
            self._package_linux_windows(url)

        else: # Macos
            self._package_macos(url)

    def _package_linux_windows(self, url: str):
        # In Linux, the tared file has a top-level root folder we have to trim
        try:
            get(self, url, strip_root=self.settings.os == "Linux")
        except Exception as ex:
            raise Exception(f"Failed to download MuJoCo source code from: '{url}'. Is the link reachable?") from ex

        package_include = os.path.join(self.package_folder, "include")
        package_lib = os.path.join(self.package_folder, "lib")
        package_bin = os.path.join(self.package_folder, "bin")

        build_include = os.path.join(self.build_folder, "include")
        build_lib = os.path.join(self.build_folder, "lib")
        build_bin = os.path.join(self.build_folder, "bin")

        copy(self, "*.h", build_include, package_include)
        copy(self, "*.dll", build_bin, package_bin)
        copy(self, "*.lib", build_lib, package_lib)
        copy(self, "*.a", build_lib, package_lib)
        copy(self, "*.so*", build_lib, package_lib)

    def _package_macos(self, url):
        filename = os.path.basename(url)

        try:
            download(self, url, filename)
        except Exception as ex:
            raise Exception(f"Failed to download MuJoCo source code from: '{url}'. Is the link reachable?") from ex

        disk = None

        try:
            disk, mount_points = MujocoConan._mount_dmg_with_disk(filename)

            if not mount_points:
                self.output.error(f"No mount points found at {filename}, downloaded from {url}")

            self.output.info(f"Mounted DMG at: {', '.join(mount_points)}")

            possible_framework_dirs = [
                os.path.join(d, "mujoco.framework")
                for d in mount_points
            ]

            framework_dir = next((d for d in possible_framework_dirs if os.path.isdir(d)), None)
            if framework_dir is None:
                self.output.error("Failed to find any mount that contains the folder 'mujoco.framework'")

            # Copy the headers to the expected include/mujoco folder
            headers_dir = os.path.join(framework_dir, "Versions", "A", "Headers")
            package_include_mujoco = os.path.join(self.package_folder, "include", "mujoco")
            copy(self, "*.h", headers_dir, package_include_mujoco)

            # Copy the entire mujoco.framework dir
            framework_dst = os.path.join(self.package_folder, "lib", "mujoco.framework")
            shutil.copytree(framework_dir, framework_dst, dirs_exist_ok=True)

        finally:
            os.unlink(filename)
            if disk:
                self.output.info(f"Detaching disk: {disk}")
                try:
                    self._detach_disk(disk)
                except Exception as e:
                    self.output.error(f"Failed to detach disk: {e}")


    def package_info(self):
        if self.settings.os == "Macos":
            self.cpp_info.frameworkdirs = [os.path.join(self.package_folder, "lib")]
            self.cpp_info.frameworks = ["mujoco"]
        else:
            self.cpp_info.libs = ["mujoco"]

    @staticmethod
    def _mount_dmg_with_disk(dmg_path):
        """
        Mount the DMG using hdiutil with the -plist flag.
        Returns a tuple (disk_identifier, [mount_points]).
        """
        result = subprocess.run(
            ["hdiutil", "attach", dmg_path, "-nobrowse", "-plist"],
            capture_output=True, check=True
        )
        plist_data = plistlib.loads(result.stdout)
        # The disk identifier is available in one of the system-entities;
        # often, the first entity has a "dev-entry" key.
        disk_identifier = None
        mount_points = []
        for entity in plist_data.get("system-entities", []):
            if not disk_identifier and "dev-entry" in entity:
                disk_identifier = entity["dev-entry"]
            mp = entity.get("mount-point")
            if mp:
                mount_points.append(mp)
        return disk_identifier, mount_points

    @staticmethod
    def _detach_disk(disk):
        """Detach the disk using hdiutil detach."""
        subprocess.run(["hdiutil", "detach", disk], check=True)
