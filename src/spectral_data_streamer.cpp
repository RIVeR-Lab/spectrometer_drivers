#include <math.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include "ftd2xx.h"
#include <ros/ros.h>
#include "libft4222.h"
#include <ros/console.h>
#include "spectrometer_drivers/Spectra.h"
#include "spectrometer_drivers/Integration.h"
// IbsenLinuxExample.cpp modified by Gary Lvov and Nathaniel Hanson
class IbsenDriver
{
public:
    IbsenDriver(ros::NodeHandle *nh)
    {
        nh->param<float>("integration_time", integrationTime, 10);
        nh->getParam("wavelength_range", wavelength_range);
        nh->getParam("min_wavelength", minWave);
        nh->getParam("max_wavelength", maxWave);
        pub = nh->advertise<spectrometer_drivers::Spectra>("spectral_data", 10);
        ros::ServiceServer service = nh->advertiseService("set_integration_time", &IbsenDriver::UpdateIntegrationCallService, this);
        // Find device handles to use here
        std::vector<int> possibleHandles = ListFtUSBDevices();
        int startIndex = this->TestDevices(possibleHandles, wavelength_range);
        this->ftHandleCS0 = (FT_HANDLE)NULL;
        this->ftHandleCS1 = (FT_HANDLE)NULL;
        // Open a connection to the spectrometer based on description for CS1.
        ftStatus = FT_OpenEx((PVOID)(uintptr_t)possibleHandles[startIndex], FT_OPEN_BY_LOCATION, &this->ftHandleCS1);
        if (ftStatus != FT_OK)
        {
            ROS_ERROR("\nFT_OpenEx failed (error %d)\n", (int)ftStatus);
        }
        else
        {
            ROS_INFO("\nDevice successfully opened with code: %d", (int)ftStatus);
        }
        // Open a connection to the spectrometer based on description for CS0.
        ftStatus = FT_OpenEx((PVOID)(uintptr_t)possibleHandles[startIndex+1], FT_OPEN_BY_LOCATION, &this->ftHandleCS0);
        if (ftStatus != FT_OK)
        {
            ROS_ERROR("\nFT_OpenEx failed (error %d)\n", (int)ftStatus);
        }
        else
        {
            ROS_INFO("\nDevice successfully opened with code: %d", (int)ftStatus);
        }
        //-------------------------------------------------------------------------------------------------------------
        // SET LATENCY TIMER
        //-------------------------------------------------------------------------------------------------------------
        UCHAR UcLatency = 20;
        ftStatus = FT_SetLatencyTimer(this->ftHandleCS1, UcLatency);
        //-------------------------------------------------------------------------------------------------------------
        // SET USB PARAMETERS
        //-------------------------------------------------------------------------------------------------------------
        ULONG OutTransferSize = 4096;
        ULONG InTransferSize = 65536;
        ftStatus = FT_SetUSBParameters(this->ftHandleCS1, InTransferSize, OutTransferSize);
        //-------------------------------------------------------------------------------------------------------------
        // SET CLOCK  - 60 MHz
        //-------------------------------------------------------------------------------------------------------------
        FT4222_SetClock(this->ftHandleCS1, SYS_CLK_60);
        //-------------------------------------------------------------------------------------------------------------
        // MASTER SPI INITIALIZE
        // SPI Status
        FT4222_STATUS ft4222Status;
        // Initialize the FT4222H as a master for USB to SPI bridge functionality.
        /*
        Handle,
        One bit at a time data stream
        Divide clock by 4 resulting in 15MHz SPI clock.
        Polarity = 0
        Phase = 1
        Slave Select SS (Triggered low) SS1O
        */
        ft4222Status =
            FT4222_SPIMaster_Init(this->ftHandleCS0, SPI_IO_SINGLE, CLK_DIV_4, CLK_IDLE_LOW, CLK_TRAILING, 2);
        if (FT4222_OK != ft4222Status)
        {
            ROS_ERROR("Init FT4222 as SPI master device failed!\n");
        }
        else
        {
            // Print the pointer of the Read write register handle, if open correctly.
            ROS_DEBUG("\nPointer of Register parameter handle, CS0 in DISB HW manual : %p", this->ftHandleCS0);
        }
        // Initialize the FT4222H as a master for USB to SPI bridge functionality.
        /*
        Handle,
        One bit at a time data stream
        Divide clock by 4 resulting in 15MHz SPI clock.
        Polarity = 0
        Phase = 1
        Slave Select SS (Triggered low) SS0O
        */
        ft4222Status =
            FT4222_SPIMaster_Init(this->ftHandleCS1, SPI_IO_SINGLE, CLK_DIV_4, CLK_IDLE_LOW, CLK_TRAILING, 3);
        if (FT4222_OK != ft4222Status)
        {
            ROS_ERROR("Init FT4222 as SPI master device failed!\n");
        }
        else
        {
            // Print the pointer of the Bulk data handle, if open correctly.
            ROS_DEBUG("\nPointer of Bulk data handle, CS1 in DISB HW manual: %p", this->ftHandleCS1);
        }
        ROS_INFO("\n");
        // Perform a single read of register 1, and both print the value to the terminal and example.txt
        uint16_t PCB_SN = this->ReadRegister(this->ftHandleCS0, 1);
        ROS_INFO("\nPCB SERIAL NUMBER %d", PCB_SN);
        // Read the HW version of the DISB board, located in register 2.
        uint16_t HardwareVersion = this->ReadRegister(this->ftHandleCS0, 2);
        ROS_INFO("\nHW_TYPE %d", HardwareVersion);
        // Read the FW version of the DISB board, located in register 3.
        uint16_t FirmwareVersion = this->ReadRegister(this->ftHandleCS0, 3);
        ROS_INFO("\nFIRMWARE %d", FirmwareVersion);
        // Read the Detector type, located in register 4.
        uint16_t DetectorType = this->ReadRegister(this->ftHandleCS0, 4);
        ROS_INFO("\nDetector type: %d", DetectorType);
        // Read the number of pixels per image, located in register 5.
        uint16_t PixelPerImage = this->ReadRegister(this->ftHandleCS0, 5);
        ROS_INFO("\nPixel per Image: %d", PixelPerImage);
        // Read the number of characters used for calibration coefficients, located in register 6.
        uint16_t NumberofCaliChars = this->ReadRegister(this->ftHandleCS0, 6);
        ROS_INFO("\nNumber of characters of calibration coefficients: %d ", NumberofCaliChars);
        // Read the wavelength calibration coefficients, via register 7.
        // When register 6 has been read the pointer is directed toward the first address of register 7.
        // Reading register 7 automatically increments the pointer.
        ROS_INFO("\nCalibration coefficients: \n");
        // string CombinedCalibrationChars;
        double ConvertedCalibrationCoefficients[NumberofCaliChars / 14];
        for (int i = 0; i < NumberofCaliChars / 14; i++)
        {
            char CombinedCalibrationChars[14];
            for (int j = 0; j < 14; j++)
            {
                CombinedCalibrationChars[j] = this->ReadRegister(this->ftHandleCS0, 7);
                // CombinedCalibrationChars = CombinedCalibrationChars + calibrationChars[j];
            }
            ROS_INFO("%s \n", CombinedCalibrationChars);
            ConvertedCalibrationCoefficients[i] = std::strtod(CombinedCalibrationChars, NULL);
        }
        // Read the temperature register number 11.
        uint16_t Temperature = this->ReadRegister(this->ftHandleCS0, 11);
        ROS_INFO("Starting temperature measured: %d", Temperature);
        // Read the Trigger delay, located in register 13 and 14.
        uint16_t TriggerDelayLSB = this->ReadRegister(this->ftHandleCS0, 13);
        uint16_t TriggerDelayMSB = this->ReadRegister(this->ftHandleCS0, 14);
        uint32_t TriggerDelay = (TriggerDelayLSB << 16) | (TriggerDelayMSB & 0xffff);
        ROS_INFO("\nTrigger delay: %d", TriggerDelay);
        // Read the Amplifier Gain, located in register 15.
        uint16_t ADCGain = this->ReadRegister(this->ftHandleCS0, 15);
        ROS_INFO("\nADC Programmable Gain Amplification (PGA) number: %d", ADCGain);
        // Read the Amplifier offset, located in register 16.
        uint16_t ADCOffset = this->ReadRegister(this->ftHandleCS0, 16);
        ROS_INFO("\nADC offset number: %d", ADCOffset);
        // Read the Spectrometer serial number, located in register 22.
        uint16_t SpectrometerSN = this->ReadRegister(this->ftHandleCS0, 22);
        ROS_INFO("\nSpectrometer serial number: %d", SpectrometerSN);

        uint16_t FirstPixel = this->ReadRegister(this->ftHandleCS0, 23);
        ROS_INFO("\nFirst pixel read: %d", FirstPixel);
        // Read the last pixel to be read, located in register 24.
        uint16_t LastPixel = this->ReadRegister(this->ftHandleCS0, 24);
        ROS_INFO("\nLast pixel read: %d", LastPixel);
        // Set starting integration time
        this->SetIntegrationTime(integrationTime);
        // Create the wavelengths
        this->wavelengths = this->GenerateWavelengths(ConvertedCalibrationCoefficients, PixelPerImage);
        while (ros::ok())
        {
            // Read the first pixel to be read, located in register 23.

            //-------------------------------------------------------------------------------------------------------------------------------
            /*
            *********************************************************************
            *********************************************************************
            Trigger exposure via write to Sensor CTRL register.
            *********************************************************************
            *********************************************************************
            */
            // Writing to the Sensor Control register 8.
            // Perform soft reset of the image transfer buffer
            this->SetRegister(this->ftHandleCS0, 8, 0x10);
            // Trigger an exposure
            this->SetRegister(this->ftHandleCS0, 8, 0x01);
            /*
            *********************************************************************
            *********************************************************************
            Read spectrum from framebuffer.
            *********************************************************************
            *********************************************************************
            */
            //-------------------------------------------------------------------------------------------------------------------------------
            // Change the chip select from 0 to 1 (CS0 --> CS1) and Framesize from 8-bit bytes to 16-bit

            //The two bytes are then stitched together to form the correct pixel values.
            uint16 datalength = (LastPixel - FirstPixel + 1.0) * 2.0;
            std::vector<uint8> NewReadFrambuffer{ 0 };
            NewReadFrambuffer.resize(datalength);
            std::vector<uint8> NewWriteFrambuffer{ 0 };
            NewWriteFrambuffer.resize(datalength);
            
            //Wait for the framebuffer to be full (NumberOfPixReady = LastPixel) or for timeout after 1 second.
            clock_t StartTime = clock();
            clock_t CurrentTime = clock();
            int TimeElapsedinSec;
            uint16_t NumberOfPixReady;
            do {
                NumberOfPixReady = ReadRegister(ftHandleCS0, 12);
                CurrentTime = clock();
                TimeElapsedinSec = (CurrentTime - StartTime) / CLOCKS_PER_SEC;
            } while (NumberOfPixReady - LastPixel < 0 && TimeElapsedinSec < 1);
            if (TimeElapsedinSec >= 1)
            {
                printf("\nTimeout error when capturing spectrum\n");
            }

            uint16_t SizeTransferred;
            //Read the pixel values
            FT4222_SPIMaster_SingleReadWrite(ftHandleCS1, NewReadFrambuffer.data(), NewWriteFrambuffer.data(), datalength, &SizeTransferred, 1);

            // ROS_INFO("\nSize of transfer %d \n", SizeTransferred);
            //Stich and print the pixel values
            std::vector<float> pixels;
            spectrometer_drivers::Spectra msg;
            uint16_t SpectrometerPix;
            for (int i = 0; i < datalength; i += 2) {
                SpectrometerPix = (NewReadFrambuffer[i] << 8) | (NewReadFrambuffer[i + 1] & 0xff);
                float SpectrometerPixCov = SpectrometerPix;
                pixels.push_back(SpectrometerPixCov);
            }
            // Extract only valid pixels from the range
            if (strcmp(this->wavelength_range.c_str(),"nir") == 0) {
                std::vector<float> finalData = {pixels.begin() + 62, pixels.begin() + 190};
                std::reverse(finalData.begin(), finalData.end());
                msg.data = finalData;
            } else {
                std::reverse(pixels.begin(), pixels.end());
                msg.data = pixels;
            }
            // Grab current temperature
            msg.temp = (int)this->ReadRegister(this->ftHandleCS0, 11);
            // Set integration time here
            msg.integrationTime = this->integrationTime;
            // Set linear space wavelengths for reference/plots
            msg.wavelengths = this->wavelengths;
            pub.publish(msg);
            // Sleep for the integration time
            ros::Duration((double)this->integrationTime/(double)1000).sleep();
            ros::spinOnce();
        }
        FT4222_UnInitialize(ftHandleCS0);
        FT4222_UnInitialize(ftHandleCS1);
        FT_Close(ftHandleCS0);
        FT_Close(ftHandleCS1);
    }
    
    ~IbsenDriver()
    {
        /*
        ********************************************************************
        *********************************************************************
        Uninitialize Device Master SPI.
        *********************************************************************
        *********************************************************************
        */
        FT4222_UnInitialize(this->ftHandleCS0);
        FT4222_UnInitialize(this->ftHandleCS1);
        /*
        *********************************************************************
        *********************************************************************
        Close Device FT4222.
        *********************************************************************
        *********************************************************************
        */
        FT_Close(this->ftHandleCS0);
        FT_Close(this->ftHandleCS1);
    }

private:
    ros::Publisher pub;
    float integrationTime;
    std::string wavelength_range;
    float minWave;
    float maxWave;
    std::vector<float> wavelengths;
    FT_STATUS ftStatus;
    FT_HANDLE ftHandleCS0;
    FT_HANDLE ftHandleCS1;
    std::vector<int> ListFtUSBDevices()
    {
        DWORD numDevs = 0;
        int i;
        FT_STATUS ftStatus;
        std::vector<int> devices;
        // Check number of devices
        ftStatus = FT_CreateDeviceInfoList(&numDevs);
        printf("%d", ftStatus);
        // Cycle through the different available device, there should be 4, (A-D)
        if (ftStatus == 0)
        {
            for (i = 0; i < (int)numDevs; i++)
            {
                // Allocate space
                FT_DEVICE_LIST_INFO_NODE devInfo;
                memset(&devInfo, 0, sizeof(devInfo));
                // Get device info for device number i,
                ftStatus =
                    FT_GetDeviceInfoDetail(i, &devInfo.Flags, &devInfo.Type, &devInfo.ID, &devInfo.LocId,
                                           devInfo.SerialNumber, devInfo.Description, &devInfo.ftHandle);
                // Print some basic info about that device.
                printf("\nDevice %d: Description '%s': LocationID '%d':", i, devInfo.Description,
                       devInfo.LocId);
                if (strcmp(devInfo.Description,"FT4222 A") == 0) {
                    // Found the first device!
                    devices.push_back(devInfo.LocId);  
                }
                if (strcmp(devInfo.Description,"FT4222 B") == 0) {
                    // Found the first device!
                    devices.push_back(devInfo.LocId);  
                }
            }
        }
        else
        {
            printf("Error, No FT4222H detected.\n");
        }
        return devices;
    }
    // Use the device's wavelength calibration coefficients to generate the central wavelength associated with each pixel
    std::vector<float> GenerateWavelengths(double Calibration[], std::size_t N)
    {
        // Used to generated full wavelength spectra
        std::vector<float> xs(N);
        // Only used for NIR where 128 pixels are present
        std::vector<float> xs_extract(128);
        for (double i = 0; i < N; i++) {
            xs[i] = Calibration[0] + Calibration[1]*i + Calibration[2]*pow(i,2) + Calibration[3]*pow(i,3) + Calibration[4]*pow(i,4);
        }
        for (float i: xs)
            printf("%f",i);
        if (strcmp(this->wavelength_range.c_str(),"nir") == 0) {
            // Extract only the 128 useful pixels
            std::copy_if (xs.begin(), xs.end(), std::back_inserter(xs_extract), [](float i){return i>=900 && i<=1702;} );
            xs_extract.erase(
                std::remove(xs_extract.begin(), xs_extract.end(), 0),
                xs_extract.end());
            xs_extract.shrink_to_fit();
            xs = xs_extract;
        }
        std::reverse(xs.begin(), xs.end());
        return xs;
    }
    bool UpdateIntegrationCallService(spectrometer_drivers::Integration::Request &req,
                                      spectrometer_drivers::Integration::Response &res) {
        // Receive a request and update the integration time in milliseconds
        res.response = true;
        try
        {
            this->SetIntegrationTime(req.data);
        }
        catch(const std::exception& e)
        {
            res.response = false;
        }
        return true;
    }
    // Calculate the integration time
    // @param newTime is the integration time in miliseconds
    void SetIntegrationTime(float newTime) {
        /*
        Setting an integration time of 10 ms.
        Integration time is set in increments of 200 ns, so 10 ms => 50.000 => MSB 0x000 - LBS 0xC350
        */
        this->integrationTime = newTime;
        int useTime = this->CalculateIntegrationTime(newTime);
        // Setting integration time of Register 10 MSB.
        unsigned int lsb = useTime & 0xFFFF;
        unsigned int msb = (useTime >> 16) & 0xFFFF;
        this->SetRegister(this->ftHandleCS0, 10, msb);
        // Setting integration time of Register 9 LSB.
        this->SetRegister(this->ftHandleCS0, 9, lsb);
    }
    int CalculateIntegrationTime(float requestTime) {
        /* Requested time is the time in miliseconds
        Integration time is set in increments of 200 ns, so 10 ms => 50.000 => MSB 0x000 - LBS 0xC350
        */
        double increments = (double)requestTime * 1000000 / 200;
        return (int)increments;
    }
    // Read value of DISB register
    uint16_t ReadRegister(PVOID FThandle, int RegisterAddress)
    {
        uint8_t AppendedRegAddress = RegisterAddress * 4 + 2;
        // Create a read- and writebuffer for data to be read and written to the DISB registers.
        uint8 ReadBuffer[3], WriteBuffer[3];
        // Used as the address for which the number of bytes being read or written after each ReadWrite command
        // is registered.
        uint16 SizeTransferred;
        WriteBuffer[0] = AppendedRegAddress;
        WriteBuffer[1] = 0x0;
        WriteBuffer[2] = 0x0;
        // Perform a single read/write operation with 3 bytes
        FT4222_SPIMaster_SingleReadWrite(FThandle, ReadBuffer, WriteBuffer, 3, &SizeTransferred, 1);
        // Combine byte1 + byte2 into a 16-bit value and return it.
        uint16_t RegisterValue = (ReadBuffer[1] << 8) | (ReadBuffer[2] & 0xff);
        return RegisterValue;
    }
    // Set a new DISB Register value
    int SetRegister(PVOID FThandle, int RegisterAddress, int NewRegisterValue)
    {
        uint8_t AppendedRegAddress = RegisterAddress * 4;
        // Create a read- and writebuffer for data to be read and written to the DISB registers.
        uint8 ReadBuffer[3], WriteBuffer[3], MSB, LSB;
        // Used as the address for which the number of bytes being read or written after each ReadWrite command
        // is registered.
        uint16 SizeTransferred;
        // Split the 16-bit value into 2 bytes
        WriteBuffer[0] = AppendedRegAddress;
        MSB = NewRegisterValue >> 8;
        LSB = NewRegisterValue;
        WriteBuffer[1] = MSB;
        WriteBuffer[2] = LSB;
        // Perform a single read/write operation with 3 bytes
        FT4222_SPIMaster_SingleReadWrite(FThandle, ReadBuffer, WriteBuffer, 3, &SizeTransferred, 1);
        return 0;
    }
    // Given a series of device handles, connect to the device and extract important metadata
    std::vector<float> GetDevice(int device_loc_1, int device_loc_2) {
        //Setup status and 2 Handles for CS0 and CS1
        FT_STATUS   ftStatus;
        FT_HANDLE	ftHandleCS0 = (FT_HANDLE)NULL;
        FT_HANDLE	ftHandleCS1 = (FT_HANDLE)NULL;

        ftStatus = FT_OpenEx((PVOID)(uintptr_t)device_loc_1, FT_OPEN_BY_LOCATION, &ftHandleCS1);
        if (ftStatus != FT_OK) {
            printf("\nFT_OpenEx failed (error %d)\n", 
                (int)ftStatus);
        }
        ftStatus = FT_OpenEx((PVOID)(uintptr_t)device_loc_2, FT_OPEN_BY_LOCATION, &ftHandleCS0);
        if (ftStatus != FT_OK){
            printf("\nFT_OpenEx failed (error %d)\n", 
                (int)ftStatus);
        }
        UCHAR UcLatency = 20;
        ftStatus = FT_SetLatencyTimer(ftHandleCS1, UcLatency);
        ULONG OutTransferSize = 4096;
        ULONG InTransferSize = 65536;
        ftStatus = FT_SetUSBParameters(ftHandleCS1, InTransferSize, OutTransferSize);
        FT4222_SetClock(ftHandleCS1, SYS_CLK_60);
        FT4222_STATUS ft4222Status;
        ft4222Status = FT4222_SPIMaster_Init(ftHandleCS0, SPI_IO_SINGLE,
                        CLK_DIV_4, CLK_IDLE_LOW, 
                        CLK_TRAILING, 2);
        if (FT4222_OK != ft4222Status) {
            printf("Init FT4222 as SPI master device failed!\n");
        }
        ft4222Status = FT4222_SPIMaster_Init(ftHandleCS1, SPI_IO_SINGLE, 
                        CLK_DIV_4, CLK_IDLE_LOW, 
                        CLK_TRAILING, 3);
        if (FT4222_OK != ft4222Status) {
            printf("Init FT4222 as SPI master device failed!\n");
        }
        //Read the number of characters used for calibration coefficients, located in register 6.
        uint16_t NumberofCaliChars = ReadRegister(ftHandleCS0, 6);
        double ConvertedCalibrationCoefficients[NumberofCaliChars / 14];
        for (int i = 0; i < NumberofCaliChars / 14; i++)
            {
            char CombinedCalibrationChars[14];
            for (int j = 0; j < 14; j++)
            {
            CombinedCalibrationChars[j] = ReadRegister(ftHandleCS0, 7);
            }
            ConvertedCalibrationCoefficients[i] = std::strtod(CombinedCalibrationChars, NULL);
        }
        //Read the number of pixels per image, located in register 5.
        uint16_t PixelPerImage = ReadRegister(ftHandleCS0, 5);
        std::vector<float> wavelengths = GenerateWavelengths(ConvertedCalibrationCoefficients, PixelPerImage);
        FT4222_UnInitialize(ftHandleCS0);
        FT4222_UnInitialize(ftHandleCS1);
        FT_Close(ftHandleCS0);
        FT_Close(ftHandleCS1);
        return wavelengths;
    }

    int TestDevices(std::vector<int> devices, std::string target) {
        printf("NUMBER OF DEVICES: %d",(int)devices.size());
        if (devices.size() % 2 != 0) {
            printf("INVALID NUMBER OF DETECTED DEVICES");
            return -1;
        }
        for (int i=0; i<(int)devices.size()-1; i=i+2) {
            std::vector<float> waves = this->GetDevice(devices[i],devices[i+1]);
            int minVal = *std::min_element(waves.begin(), waves.end());
            int maxVal = *std::max_element(waves.begin(), waves.end());
            printf("%d\n", minVal);
            printf("%d\n", maxVal);
            std::string currentDevice;
            if (minVal < 460 && maxVal < 1105) {
                currentDevice = "vnir";
            } else {
                currentDevice = "nir";
            }
            // Do a final check before returning
            if (strcmp(target.c_str(),currentDevice.c_str()) == 0) {
                return i;
            }
        }
        // No matches, we shouldn't fall into this error case
        ROS_ERROR("NO MATCHES FOR DESIRED SPECTROMETER TYPE!");
        return -1;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ibsen_driver", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    IbsenDriver nc = IbsenDriver(&nh);
}