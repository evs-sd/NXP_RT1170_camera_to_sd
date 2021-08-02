This project is based on the csi_mipi_rgb_cm7 example from the SDK. Removed display and PXP, added SD card and fatfs.

How it works: a picture is taken and saved to a SD card. You can uncomment the entry in the BMP header file, but for some unknown reason, the write speed is too slow, so it's easier to add a header on a PC.