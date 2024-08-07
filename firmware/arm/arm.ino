#include <CircularBuffer.h>
#include <Servo.h>

const int N_SERVOS = 6;
const int START_BYTE = 0x29;
const int MAX_MESSAGE_SIZE = 10;
const int CMD_WRITE_POSITION = 0x01;

Servo servos[N_SERVOS];

CircularBuffer<byte, 50> incomming_buffer;
enum State
{
    STATE_SEARCH_START_BYTE = 0,
    STATE_WAIT_FOR_MESSAGE
};
State state = STATE_SEARCH_START_BYTE;

void setup()
{
    Serial.begin(115200);

    servos[0].attach(2);
    servos[1].attach(3);
    servos[2].attach(4);
    servos[3].attach(5);
    servos[4].attach(6);
    servos[5].attach(7);
}

void loop()
{
    update_communication();
}

int16_t unpack_int16(byte *buffer)
{
    int16_t value = 0;
    value |= ((uint32_t)buffer[2] << 8);
    value |= ((uint32_t)buffer[3] << 0);
    return value;
}

void update_communication()
{
    // Add data to the buffer
    while (Serial.available() > 0)
        incomming_buffer.push(Serial.read());

    // Find the start byte
    switch (state)
    {
    case STATE_SEARCH_START_BYTE:
        if (!incomming_buffer.isEmpty() && incomming_buffer.first() != START_BYTE)
            incomming_buffer.shift();
        else
            state = STATE_WAIT_FOR_MESSAGE;
        break;
    case STATE_WAIT_FOR_MESSAGE:
        if (incomming_buffer.size() >= 2 && incomming_buffer.size() >= incomming_buffer[1])
        {
            on_full_message_received();
            state = STATE_SEARCH_START_BYTE;
        }
        if (incomming_buffer.size() >= 2 && (uint8_t)incomming_buffer[1] > MAX_MESSAGE_SIZE)
        {
            // Message too big, something is wrong.
            incomming_buffer.shift();
            incomming_buffer.shift();
            state = STATE_SEARCH_START_BYTE;
        }
        break;
    }
}

void on_full_message_received()
{
    incomming_buffer.shift(); // start byte
    incomming_buffer.shift(); // message size
    incomming_buffer.shift(); // command type

    for (uint8_t i = 0; i < N_SERVOS; i++)
    {
        byte buffer[2];
        buffer[0] = incomming_buffer.shift();
        buffer[1] = incomming_buffer.shift();

        const int16_t target_position = unpack_int16(buffer);

        servos[i].writeMicroseconds(target_position);
    }
}