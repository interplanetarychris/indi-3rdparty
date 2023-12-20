/*******************************************************************************
 Copyright(c) 2019 Jasem Mutlaq. All rights reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

/* Command Response reference

ED FT I CI:
'ED *<CR><LF>*<CR><LF>*<CR><LF>*<CR><LF>'

PP:
'PP<CR><LF>* 100<CR><LF>'

PO100 TO100:
'PO100 *<CR><LF>TO100<CR><LF>*<CR><LF>'

PP TP:
'PP * 100<CR><LF>TP<CR><LF>* 600<CR><LF>'

PP TP PH TH PM TM
'PP * 100<CR><LF>TP * 700<CR><LF>PH * OFF<CR><LF>TH * OFF<CR><LF>PM * REG<CR><LF>TM<CR><LF>* HIGH<CR><LF>'

Patterns: 
- There is a * for each command
- The last item has a <CR><LF> inserted before the *


*/


#include "flirptu.h"

#include "indicom.h"

#include <libnova/sidereal_time.h>
#include <libnova/transform.h>

#include <termios.h>
#include <cmath>
#include <cstring>
#include <memory>

#include <iterator> // For std::istream_iterator
#include <regex> // For getCurrrentPTUPosition

using namespace INDI::AlignmentSubsystem;

// Single unique pointer to the driver.
static std::unique_ptr<FlirPTU> telescope_sim(new FlirPTU());

FlirPTU::FlirPTU()
{
    // Let's specify the driver version
    setVersion(0, 1); // don't forget to update drivers.xml

    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");

    // Set capabilities supported by the mount.
    // The last parameters is the number of slew rates available.
    SetTelescopeCapability(TELESCOPE_CAN_GOTO | 
                          TELESCOPE_CAN_ABORT |
                          TELESCOPE_HAS_TRACK_MODE | 
                          TELESCOPE_CAN_CONTROL_TRACK |
                          TELESCOPE_HAS_TRACK_RATE,
                           4);

    setTelescopeConnection(CONNECTION_TCP);
}

const char *FlirPTU::getDefaultName()
{
    return "FLIR PTU";
}

bool FlirPTU::initProperties()
{
    // Make sure to init parent properties first
    INDI::Telescope::initProperties();

    // How fast do we guide compared to sidereal rate
    IUFillNumber(&GuideRateN[AXIS_RA], "GUIDE_RATE_WE", "W/E Rate", "%.1f", 0, 1, 0.1, 0.5);
    IUFillNumber(&GuideRateN[AXIS_DE], "GUIDE_RATE_NS", "N/S Rate", "%.1f", 0, 1, 0.1, 0.5);
    IUFillNumberVector(&GuideRateNP, GuideRateN, 2, getDeviceName(), "GUIDE_RATE", "Guiding Rate", MOTION_TAB, IP_RW, 0,
                       IPS_IDLE);

    // Since we have 4 slew rates, let's fill them out
    IUFillSwitch(&SlewRateS[SLEW_GUIDE], "SLEW_GUIDE", "Guide", ISS_OFF);
    IUFillSwitch(&SlewRateS[SLEW_CENTERING], "SLEW_CENTERING", "Centering", ISS_OFF);
    IUFillSwitch(&SlewRateS[SLEW_FIND], "SLEW_FIND", "Find", ISS_OFF);
    IUFillSwitch(&SlewRateS[SLEW_MAX], "SLEW_MAX", "Max", ISS_ON);
    IUFillSwitchVector(&SlewRateSP, SlewRateS, 4, getDeviceName(), "TELESCOPE_SLEW_RATE", "Slew Rate", MOTION_TAB,
                       IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Add Tracking Modes. If you have SOLAR, LUNAR..etc, add them here as well.
    AddTrackMode("TRACK_SIDEREAL", "Sidereal", true);
    AddTrackMode("TRACK_CUSTOM", "Custom");

    // Add Power States for Hold and Move

    const char *PTU_TAB = "PTU Stuff";

    IUFillNumber(&ResolutionN[PTU_PAN], "PTU_PAN_RESOLUTION", "Pan Resolution (arcsec/step)", "%.4f", 0, 60.0, 1, 0);
    IUFillNumber(&ResolutionN[PTU_TILT], "PTU_TILT_RESOLUTION", "Tilt Resolution (arcsec/step)", "%.4f", 0, 60.0, 1, 0);
    IUFillNumberVector(&ResolutionNP, ResolutionN, 2, getDeviceName(), "PTU_RESOLUTION", "PTU Resolution", PTU_TAB,
                       IP_RO, 0, IPS_IDLE);

    IUFillNumber(&MinPosN[PTU_PAN], "PTU_PAN_MIN_POS", "Pan Min Position (deg)", "%.4f", -400, 400, 1, 0);
    IUFillNumber(&MinPosN[PTU_TILT], "PTU_TILT_MIN_POS", "Tilt Min Position (deg)", "%.4f", 0, 360.0, 1, 0);
    IUFillNumberVector(&MinPosNP, MinPosN, 2, getDeviceName(), "PTU_MIN_POS", "PTU Min Position", PTU_TAB, IP_RO, 0,
                       IPS_IDLE);

    IUFillNumber(&MaxPosN[PTU_PAN], "PTU_PAN_MAX_POS", "Pan Max Position (deg)", "%.4f", -400, 400, 1, 0);
    IUFillNumber(&MaxPosN[PTU_TILT], "PTU_TILT_MAX_POS", "Tilt Max Position (deg)", "%.4f", 0, 360.0, 1, 0);
    IUFillNumberVector(&MaxPosNP, MaxPosN, 2, getDeviceName(), "PTU_MAX_POS", "PTU Max Position", PTU_TAB, IP_RO, 0,
                       IPS_IDLE);

    IUFillNumber(&VdctN[0], "PTU_VOLT", "Input Voltage", "%.1f", 0, 50, 1, 0);
    IUFillNumber(&VdctN[1], "PTU_TEMP", "Temp (°F)", "%.0f", 0, 150, 1, 0);
    IUFillNumber(&VdctN[2], "PTU_TEMP_PAN", "Pan Motor Temp (°F)", "%.0f", 0, 150, 1, 0);
    IUFillNumber(&VdctN[3], "PTU_TEMP_TILT", "Tilt Motor Temp (°F)", "%.0f", 0, 150, 1, 0);
    IUFillNumberVector(&VdctNP, VdctN, 4, getDeviceName(), "PTU_VDCT", "VDCT", PTU_TAB, IP_RO, 0, IPS_IDLE);


    IUFillNumber(&ControlModeCorrectionsN[PTU_PAN], "PTU_PAN_CORRECTIONS", "Pan Corrections", "%.0f", 0, 1e6, 1, 0);
    IUFillNumber(&ControlModeCorrectionsN[PTU_TILT], "PTU_TILT_CORRECTIONS", "Tilt Corrections", "%.0f", 0, 1e6, 1, 0);
    IUFillNumberVector(&ControlModeCorrectionsNP, ControlModeCorrectionsN, 2, getDeviceName(), "PTU_CONTROL_MODE_CORRECTIONS", "Control Mode Corrections", PTU_TAB, IP_RO, 0, IPS_IDLE);


    IUFillSwitch(&PanHoldPowerS[POWER_LOW], "PAN_HOLD_POWER_LOW", "Low", ISS_OFF);
    IUFillSwitch(&PanHoldPowerS[POWER_REG], "PAN_HOLD_POWER_REG", "Regular", ISS_ON);
    IUFillSwitch(&PanHoldPowerS[2], "PAN_HOLD_POWER_OFF", "Off", ISS_OFF);
    IUFillSwitchVector(&PanHoldPowerSP, PanHoldPowerS, 3, getDeviceName(), "PTU_PAN_HOLD_POWER", "Pan Hold Power",
                       PTU_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillSwitch(&TiltHoldPowerS[POWER_LOW], "TILT_HOLD_POWER_LOW", "Low", ISS_OFF);
    IUFillSwitch(&TiltHoldPowerS[POWER_REG], "TILT_HOLD_POWER_REG", "Regular", ISS_ON);
    IUFillSwitch(&TiltHoldPowerS[2], "TILT_HOLD_POWER_OFF", "Off", ISS_OFF);
    IUFillSwitchVector(&TiltHoldPowerSP, TiltHoldPowerS, 3, getDeviceName(), "PTU_TILT_HOLD_POWER", "Tilt Hold Power",
                       PTU_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillSwitch(&PanMovePowerS[POWER_LOW], "PAN_MOVE_POWER_LOW", "Low", ISS_OFF);
    IUFillSwitch(&PanMovePowerS[POWER_REG], "PAN_MOVE_POWER_REG", "Regular", ISS_ON);
    IUFillSwitch(&PanMovePowerS[POWER_HIGH], "PAN_MOVE_POWER_HIGH", "High", ISS_OFF);
    IUFillSwitchVector(&PanMovePowerSP, PanMovePowerS, 3, getDeviceName(), "PTU_PAN_MOVE_POWER", "Pan Move Power",
                       PTU_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillSwitch(&TiltMovePowerS[POWER_LOW], "TILT_MOVE_POWER_LOW", "Low", ISS_OFF);
    IUFillSwitch(&TiltMovePowerS[POWER_REG], "TILT_MOVE_POWER_REG", "Regular", ISS_ON);
    IUFillSwitch(&TiltMovePowerS[POWER_HIGH], "TILT_MOVE_POWER_HIGH", "High", ISS_OFF);
    IUFillSwitchVector(&TiltMovePowerSP, TiltMovePowerS, 3, getDeviceName(), "PTU_TILT_MOVE_POWER", "Tilt Move Power",
                       PTU_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillSwitch(&ControlModeS[0], "CONTROL_MODE_OPENLOOP", "Open Loop", ISS_ON);
    IUFillSwitch(&ControlModeS[1], "CONTROL_MODE_ENCODER", "Encoder (Closed Loop)", ISS_OFF);
    IUFillSwitchVector(&ControlModeSP, ControlModeS, 2, getDeviceName(), "PTU_CONTROL_MODE", "Control Mode",
                       PTU_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillSwitch(&ResetAxisS[PTU_PAN], "RESET_AXIS_PAN", "Reset Pan", ISS_OFF);
    IUFillSwitch(&ResetAxisS[PTU_TILT], "RESET_AXIS_Tilt", "Reset Tilt", ISS_OFF);
    IUFillSwitch(&ResetAxisS[PTU_BOTH], "RESET_AXIS_BOTH", "Reset Both", ISS_OFF);
    IUFillSwitchVector(&ResetAxisSP, ResetAxisS, 3, getDeviceName(), "PTU_RESET_AXIS", "Reset Axis",
                       PTU_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // The mount is initially in IDLE state.
    TrackState = SCOPE_IDLE;

    // How does the mount perform parking?
    // Some mounts can handle the parking functionality internally in the controller.
    // Other mounts have no native parking support and we use INDI to slew to a particular
    // location (Equatorial or Horizontal) and then turn off tracking there and save the location to a file
    // which would be remembered in the next power cycle.
    // This is not required if there is native support in the mount controller itself.
    SetParkDataType(PARK_AZ_ALT);

    // Let init the pulse guiding properties
    initGuiderProperties(getDeviceName(), MOTION_TAB);

    // Add debug controls
    addDebugControl();

    addAuxControls();

    // Set the driver interface to indicate that we can also do pulse guiding
    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);

    // Add alignment properties
    InitAlignmentProperties(this);

    // We want to query the mount every 500ms by default. The user can override this value.
    setDefaultPollingPeriod(1000);

    // FIXME -- Add setting pan limits with PCE here

    return true;
}

bool FlirPTU::updateProperties()
{
    INDI::Telescope::updateProperties();

    if (isConnected())
    {
        defineProperty(&GuideNSNP);
        defineProperty(&GuideWENP);
        defineProperty(&GuideRateNP);

        defineProperty(&PanHoldPowerSP);
        defineProperty(&TiltHoldPowerSP);
        defineProperty(&PanMovePowerSP);
        defineProperty(&TiltMovePowerSP);
        defineProperty(&ControlModeSP);
        defineProperty(&ControlModeCorrectionsNP);

        defineProperty(&ResolutionNP);
        defineProperty(&MinPosNP);
        defineProperty(&MaxPosNP);
        defineProperty(&VdctNP);
        defineProperty(&ResetAxisSP);

        // Read the parking file, and check if we can load any saved parking information.
        if (InitPark())
        {
            // If loading parking data is successful, we just set the default parking values.
            // By default in this example, we consider parking position Az=0 and Alt=0
            SetAxis1ParkDefault(0);
            SetAxis2ParkDefault(0);
        }
        else
        {
            // Otherwise, we set all parking data to default in case no parking data is found.
            SetAxis1Park(0);
            SetAxis2Park(0);
            SetAxis1ParkDefault(0);
            SetAxis2ParkDefault(0);
        }

        // // Get Pan & Tilt Hold Power
        getPanHoldPower();
        getTiltHoldPower();

        // // Get Tilt and Pan Move Power
        getPanMovePower();
        getTiltMovePower();

        getResolution();
        getPTULimits();
        getVdct();
        getControlMode();
    }
    else
    {
        deleteProperty(GuideNSNP.name);
        deleteProperty(GuideWENP.name);
        deleteProperty(GuideRateNP.name);

        deleteProperty(PanHoldPowerSP.name);
        deleteProperty(TiltHoldPowerSP.name);
        deleteProperty(PanMovePowerSP.name);
        deleteProperty(TiltMovePowerSP.name);
        deleteProperty(ControlModeSP.name);
        deleteProperty(ResolutionNP.name);
        deleteProperty(MinPosNP.name);
        deleteProperty(MaxPosNP.name);
        deleteProperty(VdctNP.name);
        deleteProperty(ControlModeCorrectionsNP.name);
        deleteProperty(ResetAxisSP.name);
    }

    return true;
}

bool FlirPTU::Handshake()
{
    // This functin is ensure that we have communication with the mount
    // Below we send it 0x6 byte and check for 'S' in the return. Change this
    // to be valid for your driver. It could be anything, you can simply put this below
    // return readScopeStatus()
    // since this will try to read the position and if successful, then communicatoin is OK.

    // Set up command and feedback defaults ED FT I CI
    // const std::string fCMD = "ED FT I CI";
    int nbytes_read = 0, rc = 0;
    char pRES[PTU_LEN] = {0};

    // Full response from my unit:
    // '<CR><LF><CR><LF>### PAN-TILT CONTROLLER<CR><LF>### v3.3.0, (C)2010-2011 FLIR Commercial Systems, Inc., All Rights Reserved<CR><LF>Initializing...*<CR><LF>'
    // Needs more than 0.1 second timeout to read the full response

    const char* toFind = "Initializing...*";

    if ((rc = tty_read_section(PortFD, pRES, '*', PTU_TIMEOUT, &nbytes_read)) == TTY_TIME_OUT) {
        LOG_ERROR("Handshake failed. No response from FLIR PTU TCP server.");
        return false;
    }

    if (strstr(pRES, toFind) == nullptr) {
        LOGF_ERROR("Handshake failed. Invalid response: %s", pRES);
        return false;
    }

    // Manually ensure buffer is cleared and success is verified  
    if(!verifySuccessAndClearBuffer()){
        LOG_ERROR("Handshake failed. Failed to verify success and clear buffer.");
        return false;
    }

    std::string fCMD = "FT";
    rc = sendPTUCommandAndCheckResponse(fCMD, "FT", "Enable Terse Feedback", PTU_TIMEOUT);
    if (!rc) {
        readAndEmptyBufferForDebug();
        LOG_INFO("After failed status");
        return false;
    }

    LOG_INFO("After FT");
    readAndEmptyBufferForDebug();

    // Enable User Limits (to allow for 360 degree rotation)
    fCMD = "LU";
    rc = sendPTUCommandAndCheckResponse(fCMD, "LU", "Enable User Limits", PTU_TIMEOUT);
    if (!rc) {
        readAndEmptyBufferForDebug();
        LOG_INFO("After failed status");
        return false;
    }

    // Enable Continuous Pan Rotation - requires User Limits to be enabled
    fCMD = "PCE";
    rc = sendPTUCommandAndCheckResponse(fCMD, "PCE", "Enable Continuous Pan Rotation", PTU_TIMEOUT);
    if (!rc) {
        readAndEmptyBufferForDebug();
        LOG_INFO("After failed status");
        return false;
    }

    fCMD = "PP0";
    rc = sendPTUCommandAndCheckResponse(fCMD, "PP0", "Reset Pan Position", PTU_TIMEOUT);
    if (!rc) { // FIXME - Note this can fail if the mount has been factor reset, but the axes have not been reset - Gives error "! Axis Error". Proper response is probably to RT/RP/RE
    // Appears to give a timeout because its looking for a * delimeter,  but there is none because its a "!" error
        readAndEmptyBufferForDebug();
        LOG_INFO("PP0 Failed");
        return false;
    }
    LOG_INFO("After PP0 command");
    readAndEmptyBufferForDebug();


    // bool rc = sendPTUCommand(fCMD, res, "Handshake", PTU_TIMEOUT);
    // bool rc = sendPTUCommandAndCheckResponse(fCMD, pRES, "Handshake", PTU_TIMEOUT);

    // bool rc = sendCommand(cmd, res, 1, 1);
    // if (rc == false)
    //     return false;

    return true;
}

bool FlirPTU::ReadScopeStatus()
{

    // Here we read the mount position, pier side, any status of interest.
    // This is called every POLLMS milliseconds (default 1000, but our driver set the default to 500)

    // // For example, it could be a command like this
    // char cmd[PTU_LEN] = {0}, res[PTU_LEN] = {0};
    // if (sendCommand("GetCoordinates", res) == false)
    //     return false;

    // double currentRA = 0, currentDE = 0;
    // // Assuming we get response as RA:DEC (Hours:Degree) e.g. "12.4:-34.6"
    // sscanf(res, "%lf:%lf", &currentRA, &currentDE);

    // char RAStr[PTU_LEN] = {0}, DecStr[PTU_LEN] = {0};
    // fs_sexa(RAStr, currentRA, 2, 3600);
    // fs_sexa(DecStr, currentDE, 2, 3600);
    // LOGF_DEBUG("Current RA: %s Current DEC: %s", RAStr, DecStr);

    // NewRaDec(currentRA, currentDE);

    // // E.g. get pier side as well
    // // assuming we need to send 3-bytes 0x11 0x22 0x33 to get the pier side, which is always 1 byte as 0 (EAST) or 1 (WEST)
    // cmd[0] = 0x11;
    // cmd[1] = 0x22;
    // cmd[2] = 0x33;

    // // Let us not forget to reset res buffer by zeroing it out
    // memset(res, 0, PTU_LEN);
    // if (sendCommand(cmd, res, 3, 1))
    // {
    //     setPierSide(res[0] == 0 ? PIER_EAST : PIER_WEST);
    // }

    getVdct();
    
    getControlModeCorrections();

    if(!getPTUPosition()){
        LOG_ERROR("getCurrentPTUPosition failed.");
        return false;
    }

    return true;
}

bool FlirPTU::Goto(double RA, double DE)
{
    char cmd[PTU_LEN] = {0}, res[PTU_LEN] = {0};

    // Assuming the command is in this format: sendCoords RA:DE
    snprintf(cmd, PTU_LEN, "sendCoords %g:%g", RA, DE);
    // Assuming response is 1-byte with '1' being OK, and anything else being failed.
    if (sendCommand(cmd, res, -1, 1) == false)
        return false;

    if (res[0] != '1')
        return false;

    TrackState = SCOPE_SLEWING;

    char RAStr[PTU_LEN] = {0}, DecStr[PTU_LEN] = {0};
    fs_sexa(RAStr, RA, 2, 3600);
    fs_sexa(DecStr, DE, 2, 3600);
    LOGF_INFO("Slewing to RA: %s - DEC: %s", RAStr, DecStr);
    return true;
}

bool FlirPTU::Sync(double RA, double DE)
{
    char cmd[PTU_LEN] = {0}, res[PTU_LEN] = {0};

    // Assuming the command is in this format: syncCoords RA:DE
    snprintf(cmd, PTU_LEN, "syncCoords %g:%g", RA, DE);
    // Assuming response is 1-byte with '1' being OK, and anything else being failed.
    if (sendCommand(cmd, res, -1, 1) == false)
        return false;

    if (res[0] != '1')
        return false;

    NewRaDec(RA, DE);

    return true;
}

bool FlirPTU::Park()
{
    // Send command for parking here
    TrackState = SCOPE_PARKING;
    LOG_INFO("Parking telescope in progress...");
    return true;
}

bool FlirPTU::UnPark()
{
    SetParked(false);
    return true;
}

bool FlirPTU::getResolution()
{
    float panResolution  = getFloatResponse("PR", "updatePanResolution");
    float tiltResolution = getFloatResponse("TR", "updateTiltResolution");

    if (std::isnan(panResolution || std::isnan(tiltResolution))) {
        LOG_ERROR("Failed to get Pan/Tilt Resolution.");
        ResolutionNP.s = IPS_ALERT;
        IDSetNumber(&ResolutionNP, nullptr);
        return false;
    }

    // Assuming PanResolutionN[0] and PanResolutionN[1] are the fields to be updated
    ResolutionN[PTU_PAN].value  = panResolution; // Update the first element
    ResolutionN[PTU_TILT].value = tiltResolution; // Update the second element

    ResolutionNP.s = IPS_OK;
    IDSetNumber(&ResolutionNP, nullptr);

    LOGF_INFO("Resolution updated to: Pan: %f Tilt: %f", panResolution, tiltResolution);
    return true;
}

bool FlirPTU::getPTULimits()
{
    float panMin  = getFloatResponse("PN", "updatePanMin");
    float panMax  = getFloatResponse("PX", "updatePanMax");
    float tiltMin = getFloatResponse("TN", "updateTiltMin");
    float tiltMax = getFloatResponse("TX", "updateTiltMax");

    if (std::isnan(panMin) || std::isnan(panMax) || std::isnan(tiltMin) || std::isnan(tiltMax)) {
        LOG_ERROR("Failed to get Pan/Tilt Limits.");
        MinPosNP.s = IPS_ALERT;
        MaxPosNP.s = IPS_ALERT;
        IDSetNumber(&MinPosNP, nullptr);
        IDSetNumber(&MaxPosNP, nullptr);
        return false;
    }

    MinPosN[PTU_PAN].value = (panMin * PanRes) / 3600.0;
    MinPosN[PTU_TILT].value = (tiltMin * TiltRes) / 3600.0;
    MaxPosN[PTU_PAN].value = (panMax * PanRes) / 3600.0;
    MaxPosN[PTU_TILT].value = (tiltMax * TiltRes) / 3600.0;

    MinPosNP.s = IPS_OK;
    MaxPosNP.s = IPS_OK;
    IDSetNumber(&MinPosNP, nullptr);
    IDSetNumber(&MaxPosNP, nullptr);

    LOGF_INFO("Limits updated: Pan Min: %f, Pan Max: %f, Tilt Min: %f, Tilt Max: %f", panMin, panMax, tiltMin, tiltMax);
    return true;
}

bool FlirPTU::getControlModeCorrections()
{
    int panCorrects  = getIntResponse("CPEC", "updatePanCorrects");
    int tiltCorrects = getIntResponse("CTEC", "updateTiltCorrects");

    if (std::isnan(panCorrects || std::isnan(tiltCorrects))) {
        LOG_ERROR("Failed to get Pan/Tilt Corrections.");
        ControlModeCorrectionsNP.s = IPS_ALERT;
        IDSetNumber(&ControlModeCorrectionsNP, nullptr);
        return false;
    }

    // Assuming PanResolutionN[0] and PanResolutionN[1] are the fields to be updated
    ControlModeCorrectionsN[PTU_PAN].value  = panCorrects; // Update the first element
    ControlModeCorrectionsN[PTU_TILT].value = tiltCorrects; // Update the second element

    if (panCorrects > 0 || tiltCorrects > 0) {
        ControlModeCorrectionsNP.s = IPS_ALERT;
    } else {
        ControlModeCorrectionsNP.s = IPS_OK;
    }

    if (panCorrects != lastPanCorrects || tiltCorrects != lastTiltCorrects) {
        LOGF_WARN("Control mode corrections: Pan: +%i Tilt: +%i", panCorrects-lastPanCorrects, tiltCorrects-lastTiltCorrects);
        lastPanCorrects = panCorrects;
        lastTiltCorrects = tiltCorrects;
    }

    IDSetNumber(&ControlModeCorrectionsNP, nullptr);

    return true;
}

bool FlirPTU::resetPTUAxis(int axisIndex)
{
    std::string response;
    bool commandSuccess = true;
    std::string command;
    std::string expectedResponse;
    std::string logContext;

    switch (axisIndex) {
    case PTU_PAN:
        command = "RP";
        expectedResponse = "!P!P*";
        logContext = "Pan Axis";
        break;
    case PTU_TILT:
        command = "RT";
        expectedResponse = "!T!T*";
        logContext = "Tilt Axis";
        break;
    case PTU_BOTH:
        command = "RE";
        expectedResponse = "!T!T!P!P*";
        logContext = "Both Axes";
        break;
    default:
        LOGF_ERROR("Invalid axis selection: %i", axisIndex);
        return false;
    }

    LOGF_INFO("Resetting %s...", logContext.c_str());
    ResetAxisSP.s = IPS_BUSY;
    IDSetSwitch(&ResetAxisSP, nullptr);

    commandSuccess = sendPTUCommandAndReadResponse(command, response, logContext.c_str(), 60) && response == expectedResponse;

    if (!commandSuccess) {
        LOG_ERROR("Reset Axis failed.");
        ResetAxisSP.s = IPS_ALERT;
        IUResetSwitch(&ResetAxisSP);
        ResetAxisS[axisIndex].s = ISS_OFF;
        IDSetSwitch(&ResetAxisSP, nullptr);
        return false;
    }

    ResetAxisSP.s = IPS_OK;
    IUResetSwitch(&ResetAxisSP);
    IDSetSwitch(&ResetAxisSP, nullptr);

    LOGF_INFO("Resetting %s successful.", logContext.c_str());

    return true;        
}


bool FlirPTU::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Guide Rate
        if (strcmp(name, "GUIDE_RATE") == 0)
        {
            IUUpdateNumber(&GuideRateNP, values, names, n);
            GuideRateNP.s = IPS_OK;
            IDSetNumber(&GuideRateNP, nullptr);
            return true;
        }

        // For guiding pulse, let's pass the properties up to the guide framework
        if (strcmp(name, GuideNSNP.name) == 0 || strcmp(name, GuideWENP.name) == 0)
        {
            processGuiderProperties(name, values, names, n);
            return true;
        }
    }

    // Otherwise, send it up the chains to INDI::Telescope to process any further properties
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

bool FlirPTU::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Slew Rate
        if (strcmp(name, "TELESCOPE_SLEW_RATE") == 0)
        {
            IUUpdateSwitch(&SlewRateSP, states, names, n);
            SlewRateSP.s = IPS_OK;
            IDSetSwitch(&SlewRateSP, nullptr);
            return true;
        }

        // Power States
        if (strcmp(name, "PTU_PAN_HOLD_POWER") == 0)
        {
            int rc = 0;

            int prevIndex = IUFindOnSwitchIndex(&PanHoldPowerSP);
            IUUpdateSwitch(&PanHoldPowerSP, states, names, n);
            int currIndex = IUFindOnSwitchIndex(&PanHoldPowerSP);

            switch (currIndex)
            {
            case POWER_LOW:
                rc = sendPTUCommandAndCheckResponse("PHL", "PHL", "Pan Hold Power Low");
                break;
            case POWER_REG:
                rc = sendPTUCommandAndCheckResponse("PHR", "PHR", "Pan Hold Power Regular");
                break;
            case 2:
                rc = sendPTUCommandAndCheckResponse("PHO", "PHO", "Pan Hold Power Off");
                break;
            }

            if(!rc){
                LOG_ERROR("setPanHoldPower failed.");
                PanHoldPowerSP.s = IPS_ALERT;
                PanHoldPowerS[prevIndex].s = ISS_ON;
                IDSetSwitch(&PanHoldPowerSP, nullptr);
                return false;
            }   

            PanHoldPowerSP.s = IPS_OK;
            IDSetSwitch(&PanHoldPowerSP, nullptr);

            return true;
        }

        if (strcmp(name, "PTU_TILT_HOLD_POWER") == 0)
        {
            int rc = 0;

            int prevIndex = IUFindOnSwitchIndex(&TiltHoldPowerSP);
            IUUpdateSwitch(&TiltHoldPowerSP, states, names, n);
            int currIndex = IUFindOnSwitchIndex(&TiltHoldPowerSP);

            switch (currIndex)
            {
            case POWER_LOW:
                rc = sendPTUCommandAndCheckResponse("THL", "THL", "Tilt Hold Power Low");
                break;
            case POWER_REG:
                rc = sendPTUCommandAndCheckResponse("THR", "THR", "Tilt Hold Power Regular");
                break;
            case 2:
                rc = sendPTUCommandAndCheckResponse("THO", "THO", "Tilt Hold Power Off");
                break;
            }

            if(!rc){
                LOG_ERROR("setTiltHoldPower failed.");
                TiltHoldPowerSP.s = IPS_ALERT;
                TiltHoldPowerS[prevIndex].s = ISS_ON;
                IDSetSwitch(&TiltHoldPowerSP, nullptr);
                return false;
            }   

            TiltHoldPowerSP.s = IPS_OK;
            IDSetSwitch(&TiltHoldPowerSP, nullptr);

            return true;
        }

        if (strcmp(name, "PTU_PAN_MOVE_POWER") == 0)
        {
            int rc = 0;

            int prevIndex = IUFindOnSwitchIndex(&PanMovePowerSP);
            IUUpdateSwitch(&PanMovePowerSP, states, names, n);
            int currIndex = IUFindOnSwitchIndex(&PanMovePowerSP);

            switch (currIndex)
            {
            case POWER_LOW:
                rc = sendPTUCommandAndCheckResponse("PML", "PML", "Pan Move Power Low");
                break;
            case POWER_REG:
                rc = sendPTUCommandAndCheckResponse("PMR", "PMR", "Pan Move Power Regular");
                break;
            case POWER_HIGH:
                rc = sendPTUCommandAndCheckResponse("PMH", "PMH", "Pan Move Power High");
                break;
            }

            if(!rc){
                LOG_ERROR("setPanMovePower failed.");
                PanMovePowerSP.s = IPS_ALERT;
                PanMovePowerS[prevIndex].s = ISS_ON;
                IDSetSwitch(&PanMovePowerSP, nullptr);
                return false;
            }   

            PanMovePowerSP.s = IPS_OK;
            IDSetSwitch(&PanMovePowerSP, nullptr);

            return true;
        }

        if (strcmp(name, "PTU_TILT_MOVE_POWER") == 0)
        {
            int rc = 0;

            int prevIndex = IUFindOnSwitchIndex(&TiltMovePowerSP);
            IUUpdateSwitch(&TiltMovePowerSP, states, names, n);
            int currIndex = IUFindOnSwitchIndex(&TiltMovePowerSP);

            switch (currIndex)
            {
            case POWER_LOW:
                rc = sendPTUCommandAndCheckResponse("TML", "TML", "Tilt Move Power Low");
                break;
            case POWER_REG:
                rc = sendPTUCommandAndCheckResponse("TMR", "TMR", "Tilt Move Power Regular");
                break;
            case POWER_HIGH:
                rc = sendPTUCommandAndCheckResponse("TMH", "TMH", "Tilt Move Power High");
                break;
            case POWER_OFF:
                rc = sendPTUCommandAndCheckResponse("TMO", "TMO", "Tilt Move Power Off");
                break;
            }

            if(!rc){
                LOG_ERROR("setTiltMovePower failed.");
                TiltMovePowerSP.s = IPS_ALERT;
                TiltMovePowerS[prevIndex].s = ISS_ON;
                IDSetSwitch(&TiltMovePowerSP, nullptr);
                return false;
            }   

            TiltMovePowerSP.s = IPS_OK;
            IDSetSwitch(&TiltMovePowerSP, nullptr);

            return true;
        }

        if (strcmp(name, "PTU_CONTROL_MODE") == 0)
        {
            int rc = 0;

            int prevIndex = IUFindOnSwitchIndex(&ControlModeSP);
            IUUpdateSwitch(&ControlModeSP, states, names, n);
            int currIndex = IUFindOnSwitchIndex(&ControlModeSP);

            switch (currIndex)
            {
            case 0:
                rc = sendPTUCommandAndCheckResponse("COL", "COL", "Set open loop control mode");
                break;
            case 1:
                rc = sendPTUCommandAndCheckResponse("CEC", "CEC", "Set encoder correction mode");
                break;
            }

            if(!rc){
                LOG_ERROR("setControlMode Failed.");
                ControlModeSP.s = IPS_ALERT;
                ControlModeS[prevIndex].s = ISS_ON;
                IDSetSwitch(&ControlModeSP, nullptr);
                return false;
            }   

            ControlModeSP.s = IPS_OK;
            IDSetSwitch(&ControlModeSP, nullptr);

            return true;
        }

        // Reset PTU Axes
        if (strcmp(name, "PTU_RESET_AXIS") == 0)
        {
            IUUpdateSwitch(&ResetAxisSP, states, names, n);
            int currIndex = IUFindOnSwitchIndex(&ResetAxisSP);

            bool success = resetPTUAxis(currIndex);

            return success;
        }
    }

    // Otherwise, send it up the chains to INDI::Telescope to process any further properties
    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool FlirPTU::Abort()
{
    // Example of a function call where we expect no respose
    return sendCommand("AbortMount");
}

bool FlirPTU::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    INDI_UNUSED(dir);
    INDI_UNUSED(command);
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return false;
    }

    // Implement here the actual calls to do the motion requested
    return true;
}

bool FlirPTU::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    INDI_UNUSED(dir);
    INDI_UNUSED(command);
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return false;
    }

    // Implement here the actual calls to do the motion requested
    return true;
}

IPState FlirPTU::GuideNorth(uint32_t ms)
{
    INDI_UNUSED(ms);

    // Implement here the actual calls to do the motion requested
    return IPS_BUSY;
}

IPState FlirPTU::GuideSouth(uint32_t ms)
{
    INDI_UNUSED(ms);

    // Implement here the actual calls to do the motion requested
    return IPS_BUSY;
}

IPState FlirPTU::GuideEast(uint32_t ms)
{
    INDI_UNUSED(ms);

    // Implement here the actual calls to do the motion requested
    return IPS_BUSY;
}

IPState FlirPTU::GuideWest(uint32_t ms)
{
    INDI_UNUSED(ms);

    // Implement here the actual calls to do the motion requested
    return IPS_BUSY;
}

bool FlirPTU::updateLocation(double latitude, double longitude, double elevation)
{
    INDI_UNUSED(elevation);
    INDI_UNUSED(latitude);
    INDI_UNUSED(longitude);
    // JM: INDI Longitude is 0 to 360 increasing EAST. libnova East is Positive, West is negative
    // m_GeographicLocation.lng = longitude;

    // if (m_GeographicLocation.lng > 180)
    //     m_GeographicLocation.lng -= 360;
    // m_GeographicLocation.lat = latitude;

    // // Implement here the actual calls to the controller to set the location if supported.

    // // Inform the client that location was updated if all goes well
    // LOGF_INFO("Location updated: Longitude (%g) Latitude (%g)", m_GeographicLocation.lng, m_GeographicLocation.lat);

    return true;
}

bool FlirPTU::SetCurrentPark()
{
    // Depending on the parking type defined initially (PARK_RA_DEC or PARK_AZ_ALT...etc) set the current
    // position AS the parking position.

    // Assumg PARK_AZ_ALT, we need to do something like this:

    // SetAxis1Park(getCurrentAz());
    // SetAxis2Park(getCurrentAlt());

    // Or if currentAz, currentAlt are defined as variables in our driver, then
    // SetAxis1Park(currentAz);
    // SetAxis2Park(currentAlt);

    return true;
}

bool FlirPTU::SetDefaultPark()
{
    // For RA_DE park, we can use something like this:

    // By default set RA to HA
    SetAxis1Park(get_local_sidereal_time(LocationN[LOCATION_LONGITUDE].value));
    // Set DEC to 90 or -90 depending on the hemisphere
    SetAxis2Park((LocationN[LOCATION_LATITUDE].value > 0) ? 90 : -90);

    // For Az/Alt, we can use something like this:

    // Az = 0
    SetAxis1Park(0);
    // Alt = 0
    SetAxis2Park(0);

    return true;
}

bool FlirPTU::SetTrackMode(uint8_t mode)
{
    // Sidereal/Lunar/Solar..etc

    // Send actual command here to device
    INDI_UNUSED(mode);
    return true;
}

bool FlirPTU::SetTrackEnabled(bool enabled)
{
    // Tracking on or off?
    INDI_UNUSED(enabled);
    // Send actual command here to device
    return true;
}

bool FlirPTU::SetTrackRate(double raRate, double deRate)
{
    // Send actual command here to device
    INDI_UNUSED(raRate);
    INDI_UNUSED(deRate);
    return true;
}

bool FlirPTU::getControlMode()
{
    std::string pRES;

    if (!sendPTUCommandAndReadResponse("CT", pRES, "getControlMode")) {
        LOG_ERROR("getControlMode failed.");
        return false;
    }
    LOGF_INFO("Control Mode: %s", pRES.c_str());

    IUResetSwitch(&ControlModeSP);

    if (pRES == "COL") {
        ControlModeS[0].s = ISS_ON;
    } else if (pRES == "CEC") {
        ControlModeS[1].s = ISS_ON;
    } else {
        LOGF_ERROR("Invalid response: %s", pRES.c_str());
        ControlModeSP.s = IPS_ALERT;
        IDSetSwitch(&ControlModeSP, nullptr);
        return false;
    }

    ControlModeSP.s = IPS_OK;
    IDSetSwitch(&ControlModeSP, nullptr);

    return true;
}

bool FlirPTU::getPanHoldPower()
{
    std::string pRES;

    if (!sendPTUCommandAndReadResponse("PH", pRES, "getPanHoldPower")) {
        LOG_ERROR("getPanHoldPower failed.");
        return false;
    }
    LOGF_INFO("Pan Hold Power: %s", pRES.c_str());

    IUResetSwitch(&PanHoldPowerSP);

    if (pRES == "OFF") {
        PanHoldPowerSP.s = IPS_OK;
        PanHoldPowerS[2].s = ISS_ON;
    } else if (pRES == "LOW") {
        PanHoldPowerSP.s = IPS_OK;
        PanHoldPowerS[POWER_LOW].s = ISS_ON;
    } else if (pRES == "REG") {
        PanHoldPowerSP.s = IPS_OK;
        PanHoldPowerS[POWER_REG].s = ISS_ON;
    } else {
        LOGF_ERROR("Invalid response: %s", pRES.c_str());
        PanHoldPowerSP.s = IPS_ALERT;
        IDSetSwitch(&PanHoldPowerSP, nullptr);
        return false;
    }

    IDSetSwitch(&PanHoldPowerSP, nullptr);

    return true;
}


bool FlirPTU::getTiltHoldPower()
{
    std::string pRES;

    if (!sendPTUCommandAndReadResponse("TH", pRES, "getTiltHoldPower")) {
        LOG_ERROR("getTiltHoldPower failed.");
        return false;
    }

    IUResetSwitch(&TiltHoldPowerSP);

    if (pRES == "OFF") {
        TiltHoldPowerSP.s = IPS_OK;
        TiltHoldPowerS[2].s = ISS_ON;
        LOGF_INFO("Tilt Hold Power: %s", pRES.c_str());
    } else if (pRES == "LOW") {
        TiltHoldPowerSP.s = IPS_OK;
        TiltHoldPowerS[POWER_LOW].s = ISS_ON;
        LOGF_INFO("Tilt Hold Power: %s", pRES.c_str());
    } else if (pRES == "REG") {
        TiltHoldPowerSP.s = IPS_OK;
        TiltHoldPowerS[POWER_REG].s = ISS_ON;
        LOGF_INFO("Tilt Hold Power: %s", pRES.c_str());
    } else {
        LOGF_ERROR("Invalid response: %s", pRES.c_str());
        TiltHoldPowerSP.s = IPS_ALERT;
        IDSetSwitch(&TiltHoldPowerSP, nullptr);
        return false;
    }

    IDSetSwitch(&TiltHoldPowerSP, nullptr);

    return true;
}

bool FlirPTU::getPanMovePower()
{
    std::string pRES;

    if (!sendPTUCommandAndReadResponse("PM", pRES, "getPanMovePower")) {
        LOG_ERROR("getPanMovePower failed.");
        return false;
    }
    LOGF_INFO("Pan Move Power: %s", pRES.c_str());

    IUResetSwitch(&PanMovePowerSP);

    if (pRES == "OFF") {
        PanMovePowerSP.s = IPS_OK;
        PanMovePowerS[POWER_OFF].s = ISS_ON;
    } else if (pRES == "LOW") {
        PanMovePowerSP.s = IPS_OK;
        PanMovePowerS[POWER_LOW].s = ISS_ON;
    } else if (pRES == "REG") {
        PanMovePowerSP.s = IPS_OK;
        PanMovePowerS[POWER_REG].s = ISS_ON;
    } else if (pRES == "HIGH") {
        PanMovePowerSP.s = IPS_OK;
        PanMovePowerS[POWER_HIGH].s = ISS_ON;
    } else {
        LOGF_ERROR("Invalid response: %s", pRES.c_str());
        PanMovePowerSP.s = IPS_ALERT;
        IDSetSwitch(&PanMovePowerSP, nullptr);
        return false;
    }

    IDSetSwitch(&PanMovePowerSP, nullptr);

    return true;
}

bool FlirPTU::getTiltMovePower()
{
    std::string pRES;

    if (!sendPTUCommandAndReadResponse("TM", pRES, "getTiltMovePower")) {
        LOG_ERROR("getTiltMovePower failed.");
        return false;
    }
    LOGF_INFO("Tilt Move Power: %s", pRES.c_str());

    IUResetSwitch(&TiltMovePowerSP);

    if (pRES == "OFF") {
        TiltMovePowerSP.s = IPS_OK;
        TiltMovePowerS[POWER_OFF].s = ISS_ON;
    } else if (pRES == "LOW") {
        TiltMovePowerSP.s = IPS_OK;
        TiltMovePowerS[POWER_LOW].s = ISS_ON;
    } else if (pRES == "REG") {
        TiltMovePowerSP.s = IPS_OK;
        TiltMovePowerS[POWER_REG].s = ISS_ON;
    } else if (pRES == "HIGH") {
        TiltMovePowerSP.s = IPS_OK;
        TiltMovePowerS[POWER_HIGH].s = ISS_ON;
    } else {
        LOGF_ERROR("Invalid response: %s", pRES.c_str());
        TiltMovePowerSP.s = IPS_ALERT;
        IDSetSwitch(&TiltMovePowerSP, nullptr);
        return false;
    }

    IDSetSwitch(&TiltMovePowerSP, nullptr);

    return true;
}


bool FlirPTU::getPTUPosition()
{
    const std::string command = "PP TP";
    const char STOP_CHAR = '\n';
    char buffer[1024] = {0};
    int rc = 0, nbytes_read = 0;
    int Pan, Tilt;

    ensurePTUBufferEmpty();

    if (!sendPTUCommandOnly(command, "getPTUPosition")) {
        LOG_ERROR("getPTUPosition failed.");
        return false;
    }

    std::string response;
    for (int i = 0; i < 3; ++i) {
        memset(buffer, 0, sizeof(buffer));
        if ((rc = tty_read_section(PortFD, buffer, STOP_CHAR, PTU_TIMEOUT, &nbytes_read)) != TTY_OK) {
            LOGF_ERROR("Error reading response line. Result: %d", rc);
            return false;
        }
        response.append(buffer, nbytes_read);
    }

    try {
        std::regex pattern(R"(PP \* (-?\d+)\r\nTP\r\n\* (-?\d+)\r\n)");
        std::smatch matches;

        if (std::regex_search(response, matches, pattern) && matches.size() == 3) {
            Pan = std::stoi(matches[1].str());
            Tilt = std::stoi(matches[2].str());
        } else {
            LOG_WARN("Unable to parse AXES positions.");
            return false;
        }
    } catch (const std::exception& e) {
        LOGF_ERROR("Exception caught during AXES position parsing: %s", e.what());
        return false;
    }

    return true;
}

bool FlirPTU::getVdct()
{
    std::string response;

    if (!sendPTUCommandAndReadResponse("O", response, "Error reading Vdct")) {
        LOG_ERROR("Failed to get Vdct data.");
        VdctNP.s = IPS_ALERT;
        IDSetNumber(&VdctNP, nullptr);
        return false;
    }

    char* token;
    char* endptr;
    char str[response.size() + 1];
    strcpy(str, response.c_str());

    // Parse Voltage
    token = strtok(str, ",");
    if (token == NULL) {
        LOG_ERROR("Invalid Vdct data format.");
        VdctNP.s = IPS_ALERT;
        IDSetNumber(&VdctNP, nullptr);
        return false;
    }
    VdctN[0].value = strtof(token, &endptr);

    // Parse Temperatures
    for (int i = 1; i < 4; ++i) {
        token = strtok(NULL, ",");
        if (token == NULL) {
            LOG_ERROR("Invalid Vdct data format.");
            VdctNP.s = IPS_ALERT;
            IDSetNumber(&VdctNP, nullptr);
            return false;
        }
        VdctN[i].value = strtof(token, &endptr);
    }

    VdctNP.s = IPS_OK;
    IDSetNumber(&VdctNP, nullptr);

    return true;
}


// Helper function to replace CR and LF with visible representations
std::string FlirPTU::makeControlCharactersVisible(const std::string& input) {
    std::string result;
    for (char c : input) {
        switch (c) {
            case '\r':
                result += "<CR>";
                break;
            case '\n':
                result += "<LF>";
                break;
            default:
                result += c;
        }
    }
    return result;
}

bool FlirPTU::readAndEmptyBufferForDebug(char* optionalBuffer) {
    int rc = 0, nbytes_read = 0;
    const int one_byte = 1;
    std::string accumulatedData;
    char buffer[2];
    const double readTimeout = 1;

    // Initialize accumulated data with passed fragment, if any
    if (optionalBuffer != nullptr) {
        accumulatedData.assign(optionalBuffer);
    }    

    // Continuously read until timeout
    while (true) {
        rc = tty_read(PortFD, buffer, one_byte, readTimeout, &nbytes_read);
        
        if (rc == TTY_TIME_OUT) {
            break; // Exit loop on timeout
        } else if (rc != TTY_OK) {
            LOGF_ERROR("Error reading buffer. Result: %d", rc);
            return false; // Error during read
        }

        if (nbytes_read > 0) {
            accumulatedData.append(buffer, nbytes_read);
        }

        memset(buffer, 0, sizeof(buffer)); // Clear buffer for next read
    }

    // Before logging, replace CR and LF characters
    std::string visibleData = makeControlCharactersVisible(accumulatedData);

    // Log accumulated data if any
    if (!visibleData.empty()) {
        LOGF_INFO("Accumulated buffer data for debug (with control characters): '%s'", visibleData.c_str());
    }

    return true; // Buffer read and emptied successfully}
}

bool FlirPTU::verifySuccessAndClearBuffer()
{
    const std::string PTU_SUCCESS_MSG = "\r\n";
    const int PTU_SUCCESS_MSG_SIZE = PTU_SUCCESS_MSG.length();
    char pERR[PTU_SUCCESS_MSG_SIZE + 1] = {0};
    int rc = 0, nbytes_read = 0;

    rc = tty_read(PortFD, pERR, PTU_SUCCESS_MSG_SIZE, PTU_TIMEOUT, &nbytes_read);

    // Before logging, replace CR and LF characters
    std::string visibleData = makeControlCharactersVisible(pERR);

    // If read is not successful, log and return false directly
    if (rc != TTY_OK) {
        LOGF_ERROR("Error reading error message from Flir PTU TCP server. Result: %d  Size: %d  Bytes read: %d", rc, PTU_SUCCESS_MSG_SIZE, nbytes_read);
        LOGF_ERROR("Error message: '%s'", visibleData.c_str());
        return false;
    } 

    // If read is successful, check if the success message matches
    if (strcmp(pERR, PTU_SUCCESS_MSG.c_str()) == 0) {
        return ensurePTUBufferEmpty(); // True on empty buffer
    } 

    LOG_ERROR("Flir PTU TCP server command error");
    ensurePTUBufferEmpty(pERR); // Attempt to clear buffer even if message does not match
    return false; // Message does not match
}

bool FlirPTU::ensurePTUBufferEmpty(char* optionalBuffer)
{
    int rc = 0, nbytes_read = 0, one_byte = 1;
    std::string accumulatedData;
    char buffer[2];
    double readTimeout = 0.1;

    // If we're passed a fragment of data, initialize accumulated data with it
    if (optionalBuffer != nullptr) {
        size_t initialLength = strlen(optionalBuffer);
        accumulatedData.assign(optionalBuffer, initialLength);
    }    

    // Perform initial read
    rc = tty_read(PortFD, buffer, one_byte, readTimeout, &nbytes_read);
    
    if (rc == TTY_TIME_OUT) {
        return true; // Buffer was initially empty
    } else if (rc != TTY_OK) {
        LOGF_ERROR("Initial read error. Result: %d", rc);
        return false; // Error on initial read
    }

    // If data was read, accumulate and continue reading
    if (nbytes_read > 0) {
        accumulatedData.append(buffer, nbytes_read);
    }

    // Read and accumulate until TTY_TIME_OUT
    while ((rc = tty_read(PortFD, buffer, one_byte, readTimeout, &nbytes_read)) != TTY_TIME_OUT) {    
        if (rc == TTY_OK && nbytes_read > 0) {
            accumulatedData.append(buffer, nbytes_read);
        } else {
            // Handle other errors
            LOGF_ERROR("Error reading buffer while clearing. Result: %d", rc);
            return false;
        }
        memset(buffer, 0, sizeof(buffer));
    }

    // If any data was accumulated, report it
    if (!accumulatedData.empty()) {
        LOGF_ERROR("Residual buffer data: %s", accumulatedData.c_str());
    }

    return false; // Buffer was not empty initially
}

bool FlirPTU::sendPTUCommandOnly(const std::string &fpCMD, const char *errorMessage)
{
    int rc = 0, nbytes_written = 0;

    // Add the stop character to the command
    std::string fpCMDWithStopChar = fpCMD + '\r' + '\n'; // Append CR LF to command

    ensurePTUBufferEmpty(); // Clear any residual data from the buffer prior to sending the command

    // LOGF_DEBUG("fpCMD: %s", fpCMD.c_str());

    if (fpCMD.length() > PTU_LEN) {
        LOGF_ERROR("%s command exceeds maximum length of %d bytes.", errorMessage, PTU_LEN);
        return false;
    }

    if ((rc = tty_write_string(PortFD, fpCMDWithStopChar.c_str(), &nbytes_written)) != TTY_OK) {
        LOGF_ERROR("Error writing %s command to Flir PTU TCP server. Result: %d", errorMessage, rc);
        return false;
    }

    return true;
}


bool FlirPTU::sendPTUCommand(const std::string &fpCMD, char *responseBuffer, const char *errorMessage, long timeoutSeconds, char STOP_CHAR)
{
    int rc = 0, nbytes_written = 0, nbytes_read = 0;

    // Add the stop character to the command
    std::string fpCMDWithStopChar = fpCMD + '\r' + '\n'; // Append CR LF to command

    ensurePTUBufferEmpty(); // Clear any residual data from the buffer prior to sending the command

    LOGF_DEBUG("fpCMD: %s", fpCMD.c_str());

    if (fpCMD.length() > PTU_LEN) {
        LOGF_ERROR("%s command exceeds maximum length of %d bytes.", errorMessage, PTU_LEN);
        return false;
    }

    if ((rc = tty_write_string(PortFD, fpCMDWithStopChar.c_str(), &nbytes_written)) != TTY_OK) {
        LOGF_ERROR("Error writing %s command to Flir PTU TCP server. Result: %d", errorMessage, rc);
        return false;
    }

    if ((rc = tty_read_section(PortFD, responseBuffer, STOP_CHAR, timeoutSeconds, &nbytes_read)) != TTY_OK) {
        LOGF_ERROR("Error reading response from FLIR PTU TCP server for %s. Result: %d", errorMessage, rc);
        return false;
    } 
    else if (nbytes_read < 1) {
        LOGF_ERROR("Insufficient data (%d byte(s)) read from FLIR PTU TCP server for %s.", nbytes_read, errorMessage);
        LOGF_ERROR("Response: %s", responseBuffer);
        return false;
    }

    // LOGF_INFO("RES: %s", responseBuffer);

    return verifySuccessAndClearBuffer(); // Ensure buffer is cleared and success is verified
}

bool FlirPTU::sendPTUCommandAndCheckResponse(const std::string &command, const std::string &expectedResponse, const char *errorMessage, long timeoutSeconds, char STOP_CHAR)
{
    char pRES[PTU_LEN] = {0};

    if (!sendPTUCommand(command, pRES, errorMessage, timeoutSeconds, STOP_CHAR)) {
        LOG_DEBUG("sendPTUCommandAndCheckResponse failed.");
        return false;
    }

    // Check if the response matches the expected response
    if (strncmp(pRES, expectedResponse.c_str(), expectedResponse.length()) == 0) {
        return true;
    }

    // Create a substring from pRES up to the length of the expected response
    std::string pRESSubstr(pRES, expectedResponse.length());

    LOGF_DEBUG("sendPTUCommandAndCheckResponse %s failed. Response: %s Expected: %s", errorMessage, pRESSubstr.c_str(), expectedResponse.c_str());
    return false; // Not matching expected response isn't a failure, just false
}

bool FlirPTU::sendPTUCommandAndGetResponse(const std::string &command, std::string &response, const char *errorMessage, long timeoutSeconds)
{
    char pRES[MAXRBUF] = {0};

    if (!sendPTUCommand(command, pRES, errorMessage, timeoutSeconds)) {
        return false;
    }

    response.assign(pRES);

    return true;
}

bool FlirPTU::sendPTUCommandAndReadResponse(const std::string& command, std::string& responseValue, const char *errorMessage, long timeoutSeconds)
{
    const char STOP_CHAR = '\n';
    char buffer[1024] = {0};
    int rc = 0, nbytes_read = 0;

    ensurePTUBufferEmpty();

    if (!sendPTUCommandOnly(command, "sendPTUCommandAndReadResponse")) {
        LOG_ERROR("sendPTUCommandAndReadResponse failed.");
        return false;
    }

    // Read echoed command
    memset(buffer, 0, sizeof(buffer));
    if ((rc = tty_read_section(PortFD, buffer, STOP_CHAR, timeoutSeconds, &nbytes_read)) != TTY_OK) {
        LOGF_ERROR("Error reading echoed command for %s. Result: %d", errorMessage, rc);
        return false;
    }
    std::string echoedCommand(buffer, nbytes_read);

    // Remove trailing \r\n from echoed command for comparison
    echoedCommand.erase(echoedCommand.find_last_not_of("\r\n") + 1);

    if (echoedCommand != command) {
        LOGF_ERROR("Echoed command does not match sent command. Echoed: %s, Sent: %s", echoedCommand.c_str(), command.c_str());
        return false;
    }

    // Read response value
    memset(buffer, 0, sizeof(buffer));
    if ((rc = tty_read_section(PortFD, buffer, STOP_CHAR, timeoutSeconds, &nbytes_read)) != TTY_OK) {
        LOGF_ERROR("Error reading response value for %s. Result: %d", errorMessage, rc);
        return false;
    }

    responseValue = std::string(buffer, nbytes_read);
    // LOGF_INFO("Raw response value: %s", responseValue.c_str()); // Add this line for debugging    

    // Remove any leading '*' and spaces, and trailing \r\n
    size_t start = responseValue.find_first_not_of("* ");
    size_t end = responseValue.find_last_not_of("\r\n");
    if (start != std::string::npos && end != std::string::npos) {
        responseValue = responseValue.substr(start, end - start + 1);
        // LOGF_INFO("%s Response value: %s", command.c_str(), responseValue.c_str()); // FIXME get rid of these when done debugging
    } else {
        LOGF_ERROR("Invalid response format. Start: %zu, End: %zu", start, end); // Add detailed error log
        return false;
    }

    return true;
}

int FlirPTU::getIntResponse(const std::string &command, const char* errorContext)
{
    std::string response;

    if (!sendPTUCommandAndReadResponse(command, response, errorContext)) {
        LOGF_ERROR("%s command failed.", errorContext);
        return -1; // Return an error code on failure
    }

    try {
        return std::stoi(response);
    } catch (const std::exception& e) {
        LOGF_ERROR("%s: Failed to parse response as integer: %s", errorContext, e.what());
        return -1; // Return an error code if parsing fails
    }
}

float FlirPTU::getFloatResponse(const std::string &command, const char* errorContext)
{
    std::string response;

    if (!sendPTUCommandAndReadResponse(command, response, errorContext)) {
        LOGF_ERROR("%s command failed.", errorContext);
        return NAN; // Return NaN on failure
    }

    try {
        return std::stof(response);
    } catch (const std::exception& e) {
        LOGF_ERROR("%s: Failed to parse response as float: %s", errorContext, e.what());
        return NAN; // Return NaN if parsing fails
    }
}

bool FlirPTU::sendMultiPTUCommandAndGetResponse(const std::string &fpCMD, char *responseBuffer, const char *errorMessage, long timeoutSeconds)
{
 int rc = 0, nbytes_written = 0, nbytes_read = 0;
 std::string fullResponse = "";
 const char STOP_CHAR = '\n'; // Change stop char to '\n' as each response ends with <CR><LF>
 const std::string DELIM = ":";
 const std::string CR_LF = "\r\n";

 std::string fpCMDWithStopChar = fpCMD + CR_LF;
 ensurePTUBufferEmpty();

 LOGF_DEBUG("fpCMD: %s", fpCMD.c_str());
 if (fpCMD.length() > PTU_LEN) {
     LOGF_ERROR("%s command exceeds maximum length.", errorMessage);
     return false;
 }

 if ((rc = tty_write_string(PortFD, fpCMDWithStopChar.c_str(), &nbytes_written)) != TTY_OK) {
     LOGF_ERROR("Error writing command. Result: %d", rc);
     return false;
 }

 std::istringstream iss(fpCMD);
 std::string command;
 while (std::getline(iss, command, ' ')) {
     char buffer[1024] = {0};

     if ((rc = tty_read_section(PortFD, buffer, STOP_CHAR, timeoutSeconds, &nbytes_read)) != TTY_OK) {
         LOGF_ERROR("Error reading response. Result: %d", rc);
         return false;
     }

     std::string readData(buffer, nbytes_read);
     // Remove CR LF and '*' from the response
     size_t crlf_pos = readData.find(CR_LF);
     if (crlf_pos != std::string::npos) {
         readData.erase(crlf_pos);
     }
     size_t asterisk_pos = readData.find('*');
     if (asterisk_pos != std::string::npos) {
         readData.erase(0, asterisk_pos + 1);
     }

     // Trim leading and trailing spaces
     readData.erase(0, readData.find_first_not_of(" "));
     readData.erase(readData.find_last_not_of(" ") + 1);

     if (!fullResponse.empty()) {
         fullResponse += DELIM;
     }

     fullResponse += command + DELIM + readData;
 }

 // Handle the last parameter separately
 char lastParamBuffer[1024] = {0};
 if ((rc = tty_read_section(PortFD, lastParamBuffer, STOP_CHAR, timeoutSeconds, &nbytes_read)) != TTY_OK) {
     LOGF_ERROR("Error reading last parameter. Result: %d", rc);
     return false;
 }
 std::string lastParam(lastParamBuffer, nbytes_read);
 // Remove CR LF and '*' from the last parameter
 size_t crlf_pos = lastParam.find(CR_LF);
 if (crlf_pos != std::string::npos) {
     lastParam.erase(crlf_pos);
 }
 size_t asterisk_pos = lastParam.find('*');
 if (asterisk_pos != std::string::npos) {
     lastParam.erase(0, asterisk_pos + 1);
 }
 // Trim leading and trailing spaces
 lastParam.erase(0, lastParam.find_first_not_of(" "));
 lastParam.erase(lastParam.find_last_not_of(" ") + 1);

 // Check if the 'TM' command is already in the full response
 if (fullResponse.find("TM") == std::string::npos) {
     fullResponse += DELIM + lastParam;
 }

 // Copy the full response into responseBuffer
 strncpy(responseBuffer, fullResponse.c_str(), fullResponse.length() + 1);

 return true; // Successful execution
}

bool FlirPTU::sendCommand(const char * cmd, char * res, int cmd_len, int res_len)
{
    int nbytes_written = 0, nbytes_read = 0, rc = -1;

    tcflush(PortFD, TCIOFLUSH);

    if (cmd_len > 0)
    {
        char hex_cmd[PTU_LEN * 3] = {0};
        hexDump(hex_cmd, cmd, cmd_len);
        LOGF_DEBUG("CMD <%s>", hex_cmd);
        rc = tty_write(PortFD, cmd, cmd_len, &nbytes_written);
    }
    else
    {
        LOGF_DEBUG("CMD <%s>", cmd);
        rc = tty_write_string(PortFD, cmd, &nbytes_written);
    }

    if (rc != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial write error: %s.", errstr);
        return false;
    }

    if (res == nullptr)
        return true;

    if (res_len > 0)
        rc = tty_read(PortFD, res, res_len, PTU_TIMEOUT, &nbytes_read);
    else
        rc = tty_nread_section(PortFD, res, PTU_LEN, PTU_STOP_CHAR, PTU_TIMEOUT, &nbytes_read);

    if (rc != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial read error: %s.", errstr);
        return false;
    }

    if (res_len > 0)
    {
        char hex_res[PTU_LEN * 3] = {0};
        hexDump(hex_res, res, res_len);
        LOGF_DEBUG("RES <%s>", hex_res);
    }
    else
    {
        LOGF_DEBUG("RES <%s>", res);
    }

    tcflush(PortFD, TCIOFLUSH);

    return true;
}

void FlirPTU::hexDump(char * buf, const char * data, int size)
{
    for (int i = 0; i < size; i++)
        sprintf(buf + 3 * i, "%02X ", static_cast<uint8_t>(data[i]));

    if (size > 0)
        buf[3 * size - 1] = '\0';
}
