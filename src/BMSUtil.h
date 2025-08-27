
#include <Arduino.h>
#include "Logger.h"
/**
 * @class BMSUtil
 * @brief A utility class providing static methods for communicating with the Tesla BMS.
 *
 * This class encapsulates the low-level communication protocol, including CRC generation,
 * sending commands, and receiving replies. It also features a built-in retry mechanism
 * to enhance communication reliability, especially when dealing with potential serial timing
 * discrepancies. All methods are static, so this class is not intended to be instantiated.
 */
class BMSUtil
{
public:
    /**
     * @brief Generates an 8-bit CRC (Cyclic Redundancy Check) for a given data buffer.
     *
     * This function calculates a CRC-8 checksum using the polynomial 0x07. This is used
     * to ensure data integrity during communication with the BMS.
     * @param input Pointer to the byte array for which to calculate the CRC.
     * @param lenInput The number of bytes in the input array.
     * @return The calculated 8-bit CRC value.
     */
    static uint8_t genCRC(uint8_t *input, int lenInput)
    {
        uint8_t generator = 0x07;
        uint8_t crc = 0;

        for (int x = 0; x < lenInput; x++)
        {
            crc ^= input[x]; /* XOR-in the next input byte */

            for (int i = 0; i < 8; i++)
            {
                if ((crc & 0x80) != 0)
                {
                    crc = (uint8_t)((crc << 1) ^ generator);
                }
                else
                {
                    crc <<= 1;
                }
            }
        }

        return crc;
    }

    /**
     * @brief Sends a data frame over the BMS serial port.
     *
     * This function constructs and sends a command frame to the BMS. It takes the module address
     * from the first byte of the data array and sets the read/write flag based on the `isWrite`
     * parameter. For write operations, it calculates and appends a CRC to the end of the message.
     * The function also provides debug logging of the sent data if enabled.
     *
     * @param data Pointer to the byte array containing the data to send. The first byte must be the module address.
     * @param dataLen The total length of the data array.
     * @param isWrite A boolean flag indicating if the operation is a write (true) or read (false).
     */
    static void sendData(uint8_t *data, uint8_t dataLen, bool isWrite)
    {
        uint8_t orig = data[0];
        uint8_t addrByte = data[0];
        if (isWrite)
            addrByte |= 1;
        SERIALBMS.write(addrByte);
        SERIALBMS.write(&data[1], dataLen - 1); // assumes that there are at least 2 bytes sent every time. There should be, addr and cmd at the least.
        data[0] = addrByte;
        if (isWrite)
            SERIALBMS.write(genCRC(data, dataLen));

        if (Logger::isDebug())
        {
            SERIAL_CONSOLE.print("Sending: ");
            SERIAL_CONSOLE.print(addrByte, HEX);
            SERIAL_CONSOLE.print(" ");
            for (int x = 1; x < dataLen; x++)
            {
                SERIAL_CONSOLE.print(data[x], HEX);
                SERIAL_CONSOLE.print(" ");
            }
            if (isWrite)
                SERIAL_CONSOLE.print(genCRC(data, dataLen), HEX);
            SERIAL_CONSOLE.println();
        }

        data[0] = orig;
    }

    /**
     * @brief Reads a reply from the BMS serial port into a buffer.
     *
     * This function reads available bytes from the BMS serial port up to a specified maximum length.
     * If more bytes are available than `maxLen`, it reads and discards the excess to clear the buffer.
     * The received data is logged for debugging if enabled.
     *
     * @param data Pointer to the buffer where the received data will be stored.
     * @param maxLen The maximum number of bytes to read into the buffer.
     * @return The actual number of bytes read from the serial port.
     */
    static int getReply(uint8_t *data, int maxLen)
    {
        int numBytes = 0;
        if (Logger::isDebug())
            SERIAL_CONSOLE.print("Reply: ");
        while (SERIALBMS.available() && numBytes < maxLen)
        {
            data[numBytes] = SERIALBMS.read();
            if (Logger::isDebug())
            {
                SERIAL_CONSOLE.print(data[numBytes], HEX);
                SERIAL_CONSOLE.print(" ");
            }
            numBytes++;
        }
        if (maxLen == numBytes)
        {
            while (SERIALBMS.available())
                SERIALBMS.read();
        }
        if (Logger::isDebug())
            SERIAL_CONSOLE.println();
        return numBytes;
    }

    /**
     * @brief Sends a command and waits for a reply with an automatic retry mechanism.
     *
     * This function is a high-level wrapper that combines `sendData` and `getReply`. It sends a command
     * and then attempts to read a reply of a specific expected length. If the received reply length
     * does not match the expected length, it will automatically retry the entire process up to 3 times.
     * This helps to mitigate communication errors that may occur due to hardware timing imperfections 
     * on Arduino Due.
     *
     * @param data Pointer to the byte array containing the command to send.
     * @param dataLen The length of the command data array.
     * @param isWrite A boolean flag indicating if the operation is a write (true) or read (false).
     * @param retData Pointer to the buffer where the reply will be stored.
     * @param retLen The expected length of the reply.
     * @return The number of bytes received in the reply. If the retries fail, it returns the length of the last unsuccessful attempt.
     */
    static int sendDataWithReply(uint8_t *data, uint8_t dataLen, bool isWrite, uint8_t *retData, int retLen)
    {
        int attempts = 1;
        int returnedLength;
        while (attempts < 4)
        {
            sendData(data, dataLen, isWrite);
            delay(2 * ((retLen / 8) + 1));
            returnedLength = getReply(retData, retLen);
            if (returnedLength == retLen)
                return returnedLength;
            attempts++;
        }
        return returnedLength; // failed to get a proper response.
    }
};
