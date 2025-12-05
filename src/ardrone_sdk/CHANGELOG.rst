^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ardrone_sdk
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2025-12-05)
------------------
* add libjson-c-dev rosdep just added to rosdistro
* Contributors: Victor Talpaert

2.0.0 (2025-12-04)
------------------
* Directly includes Parrot ARSDK source files instead of using tarball download
* Update ardrone_sdk external dependencies
* Contributors: Victor Talpaert, vtalpaert

1.1.0 (2025-05-18)
------------------
* Compile ardrone_sdk without using the intermediate step of arsdk3 package

1.0.0 (2025-05-02)
------------------
* Export Parrot ARSDK3 as a library with all cmake confguration to use find_package 
* Includes the headers, shared libraries (.so), native unix samples and Parrot licence as required
* Contributors: Victor Talpaert
