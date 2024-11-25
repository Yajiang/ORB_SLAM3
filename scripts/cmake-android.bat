cd /d %~dp0
::set path=%cd%
::#先退出到 scripts 目录
cd ..

::#########################
set android_path=D:/Android

set android_ndk=%android_path%/android-ndk-r25b
set cmake_program=%android_path%/Sdk/cmake/3.22.1/bin/cmake
set make_program=%android_path%/Sdk/cmake/3.22.1/bin/ninja
set toolchain=clang
set toolchain_file=%android_ndk%/build/cmake/android.toolchain.cmake
set android_platform=android-30
set build_type=Release
::set build_type=Debug


setlocal enabledelayedexpansion

set prefix=build-
set android_abi_list=arm64-v8a
for %%a in (%android_abi_list%) do (
	set abi=%%a
	echo ######################################!abi!
	if not exist !prefix!!abi! md !prefix!!abi!
	cd !prefix!!abi!
	rmdir /s /q !prefix!!abi!
	%cmake_program% .. ^
					-G Ninja ^
					-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON ^
					-DCMAKE_TOOLCHAIN_FILE=%toolchain_file% ^
					-DANDROID_TOOLCHAIN=%toolchain% ^
					-DANDROID_NDK=%android_ndk% ^
					-DCMAKE_BUILD_TYPE=%build_type% ^
					-DANDROID_ABI=!abi! ^
					-DANDROID_PLATFORM=%android_platform% ^
					-DANDROID_STL=c++_shared ^
					-DCMAKE_MAKE_PROGRAM=%make_program% ^
					-DBUILD_SHARED=ON
					
  

	%cmake_program% --build . --config %build_type%

	%cmake_program% --install . --config %build_type%

	cd ..
)


::pause
::#########################
echo "build finish"
cd scripts
