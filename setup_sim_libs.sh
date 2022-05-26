#!/bin/sh

# Run this script to run unit tests & simulation on macOS (must run a bazel build before)
#
# symlinks wpilib libraries to /usr/local/lib

INSTALL_PATH="/usr/local/lib/"

ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_hal_hal-cpp_osxx86-64/osx/x86-64/shared/libwpiHal.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_hal_hal-cpp_osxx86-64/osx/x86-64/shared/libwpiHalJni.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_ntcore_ntcore-cpp_osxx86-64/osx/x86-64/shared/libntcore.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_ntcore_ntcore-cpp_osxx86-64/osx/x86-64/shared/libntcorejni.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpimath_wpimath-cpp_osxx86-64/osx/x86-64/shared/libwpimath.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpimath_wpimath-cpp_osxx86-64/osx/x86-64/shared/libwpimathjni.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpiutil_wpiutil-cpp_osxx86-64/osx/x86-64/shared/libwpiutil.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpiutil_wpiutil-cpp_osxx86-64/osx/x86-64/shared/libwpiutiljni.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_cscore_cscore-cpp_osxx86-64/osx/x86-64/shared/libcscore.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_cscore_cscore-cpp_osxx86-64/osx/x86-64/shared/libcscorejni.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpilibc_wpilibc-cpp_osxx86-64/osx/x86-64/shared/libwpilibc.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpilibc_wpilibc-cpp_osxx86-64/osx/x86-64/shared/libwpilibcjni.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpilibc_wpilibc-cpp_osxx86-64/osx/x86-64/shared/libwpilibcjni.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenix_sim_api-cpp-sim_osxx86-64/osx/x86-64/shared/libCTRE_PhoenixSim.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenix_sim_cci-sim_osxx86-64/osx/x86-64/shared/libCTRE_PhoenixCCISim.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenix_sim_simcancoder_osxx86-64/osx/x86-64/shared/libCTRE_SimCANCoder.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenix_sim_simpigeonimu_osxx86-64/osx/x86-64/shared/libCTRE_SimPigeonIMU.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenix_sim_simtalonfx_osxx86-64/osx/x86-64/shared/libCTRE_SimTalonFX.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenix_sim_simtalonsrx_osxx86-64/osx/x86-64/shared/libCTRE_SimTalonSRX.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenix_sim_simvictorspx_osxx86-64/osx/x86-64/shared/libCTRE_SimVictorSPX.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_com_ctre_phoenix_sim_wpiapi-cpp-sim_osxx86-64/osx/x86-64/shared/libCTRE_Phoenix_WPISim.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_wpilibnewcommands_wpilibnewcommands-cpp_osxx86-64/osx/x86-64/shared/libwpilibNewCommands.dylib $INSTALL_PATH
ln -sf $(pwd)/external/__bazelrio_edu_wpi_first_halsim_halsim_gui_osxx86-64/osx/x86-64/shared/libhalsim_gui.dylib $INSTALL_PATH