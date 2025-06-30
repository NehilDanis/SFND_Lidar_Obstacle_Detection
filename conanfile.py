from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout

class PCLMacExampleConan(ConanFile):
    name = "pcl_macos"
    version = "1.0"
    settings = "os", "compiler", "build_type", "arch"
    requires = ["pcl/1.15.0"]
    generators = "CMakeToolchain", "CMakeDeps"
    exports_sources = "src/*", "CMakeLists.txt"
    # or set default_options instead (preferred)
    default_options = {
        "pcl/*:with_visualization": True,
        "pcl/*:with_vtk": True
    }

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
