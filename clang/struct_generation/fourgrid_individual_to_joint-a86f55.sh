# Crash reproducer for clang version 3.8.0-2ubuntu4 (tags/RELEASE_380/final)
# Driver args: "--driver-mode=g++" "-D" "QT_CORE_LIB" "-D" "QT_GUI_LIB" "-D" "QT_NO_DEBUG" "-D" "QT_OPENGL_LIB" "-I" "/home/kyle/code/minutebots/build" "-I" "/home/kyle/code/minutebots" "-isystem" "/usr/include/qt4" "-isystem" "/usr/include/qt4/QtOpenGL" "-isystem" "/usr/include/qt4/QtGui" "-isystem" "/usr/include/qt4/QtCore" "-I" "/home/kyle/code/minutebots/src" "-I" "/home/kyle/code/minutebots/third_party" "-I" "/usr/local/include/luajit-2.0" "-I" "/usr/local/include" "-I" "/usr/include/eigen3" "-I" "/home/kyle/code/minutebots/third_party/googletest/googletest-release-1.8.0/googletest/include" "-I" "/home/kyle/code/minutebots/third_party/googletest/googletest-release-1.8.0/googletest" "-std=c++11" "-Wall" "-Wsign-compare" "-Werror" "-g" "-o" "penmp" "-fno-builtin-malloc" "-fno-builtin-calloc" "-fno-builtin-realloc" "-fno-builtin-free" "-O3" "-D" "NDEBUG" "-O3" "-D" "NDEBUG" "-o" "CMakeFiles/fourgrind_individual_to_joint.dir/src/search/fourgrid/fourgrid_individual_to_joint.cc.o" "-c" "/home/kyle/code/minutebots/src/search/fourgrid/fourgrid_individual_to_joint.cc"
# Original command:  "/usr/lib/llvm-3.8/bin/clang" "-cc1" "-triple" "x86_64-pc-linux-gnu" "-emit-obj" "-disable-free" "-disable-llvm-verifier" "-main-file-name" "fourgrid_individual_to_joint.cc" "-mrelocation-model" "static" "-mthread-model" "posix" "-fmath-errno" "-masm-verbose" "-mconstructor-aliases" "-munwind-tables" "-fuse-init-array" "-target-cpu" "x86-64" "-momit-leaf-frame-pointer" "-dwarf-column-info" "-debug-info-kind=limited" "-dwarf-version=4" "-debugger-tuning=gdb" "-coverage-file" "/home/kyle/code/minutebots/build/CMakeFiles/fourgrind_individual_to_joint.dir/src/search/fourgrid/fourgrid_individual_to_joint.cc.o" "-resource-dir" "/usr/lib/llvm-3.8/bin/../lib/clang/3.8.0" "-isystem" "/usr/include/qt4" "-isystem" "/usr/include/qt4/QtOpenGL" "-isystem" "/usr/include/qt4/QtGui" "-isystem" "/usr/include/qt4/QtCore" "-D" "QT_CORE_LIB" "-D" "QT_GUI_LIB" "-D" "QT_NO_DEBUG" "-D" "QT_OPENGL_LIB" "-I" "/home/kyle/code/minutebots/build" "-I" "/home/kyle/code/minutebots" "-I" "/home/kyle/code/minutebots/src" "-I" "/home/kyle/code/minutebots/third_party" "-I" "/usr/local/include/luajit-2.0" "-I" "/usr/local/include" "-I" "/usr/include/eigen3" "-I" "/home/kyle/code/minutebots/third_party/googletest/googletest-release-1.8.0/googletest/include" "-I" "/home/kyle/code/minutebots/third_party/googletest/googletest-release-1.8.0/googletest" "-D" "NDEBUG" "-D" "NDEBUG" "-internal-isystem" "/usr/bin/../lib/gcc/x86_64-linux-gnu/5.4.0/../../../../include/c++/5.4.0" "-internal-isystem" "/usr/bin/../lib/gcc/x86_64-linux-gnu/5.4.0/../../../../include/x86_64-linux-gnu/c++/5.4.0" "-internal-isystem" "/usr/bin/../lib/gcc/x86_64-linux-gnu/5.4.0/../../../../include/x86_64-linux-gnu/c++/5.4.0" "-internal-isystem" "/usr/bin/../lib/gcc/x86_64-linux-gnu/5.4.0/../../../../include/c++/5.4.0/backward" "-internal-isystem" "/usr/local/include" "-internal-isystem" "/usr/lib/llvm-3.8/bin/../lib/clang/3.8.0/include" "-internal-externc-isystem" "/usr/include/x86_64-linux-gnu" "-internal-externc-isystem" "/include" "-internal-externc-isystem" "/usr/include" "-O3" "-Wall" "-Wsign-compare" "-Werror" "-std=c++11" "-fdeprecated-macro" "-fdebug-compilation-dir" "/home/kyle/code/minutebots/build" "-ferror-limit" "19" "-fmessage-length" "211" "-fno-builtin-malloc" "-fno-builtin-calloc" "-fno-builtin-realloc" "-fno-builtin-free" "-fobjc-runtime=gcc" "-fcxx-exceptions" "-fexceptions" "-fdiagnostics-show-option" "-fcolor-diagnostics" "-vectorize-loops" "-vectorize-slp" "-o" "CMakeFiles/fourgrind_individual_to_joint.dir/src/search/fourgrid/fourgrid_individual_to_joint.cc.o" "-x" "c++" "/home/kyle/code/minutebots/src/search/fourgrid/fourgrid_individual_to_joint.cc"
 "/usr/lib/llvm-3.8/bin/clang" "-cc1" "-triple" "x86_64-pc-linux-gnu" "-emit-obj" "-disable-free" "-disable-llvm-verifier" "-main-file-name" "fourgrid_individual_to_joint.cc" "-mrelocation-model" "static" "-mthread-model" "posix" "-fmath-errno" "-masm-verbose" "-mconstructor-aliases" "-munwind-tables" "-fuse-init-array" "-target-cpu" "x86-64" "-momit-leaf-frame-pointer" "-dwarf-column-info" "-debug-info-kind=limited" "-dwarf-version=4" "-debugger-tuning=gdb" "-D" "QT_CORE_LIB" "-D" "QT_GUI_LIB" "-D" "QT_NO_DEBUG" "-D" "QT_OPENGL_LIB" "-D" "NDEBUG" "-D" "NDEBUG" "-O3" "-Wall" "-Wsign-compare" "-Werror" "-std=c++11" "-fdeprecated-macro" "-ferror-limit" "19" "-fmessage-length" "211" "-fno-builtin-malloc" "-fno-builtin-calloc" "-fno-builtin-realloc" "-fno-builtin-free" "-fobjc-runtime=gcc" "-fcxx-exceptions" "-fexceptions" "-fdiagnostics-show-option" "-fcolor-diagnostics" "-vectorize-loops" "-vectorize-slp" "-x" "c++" "fourgrid_individual_to_joint-a86f55.cpp"
