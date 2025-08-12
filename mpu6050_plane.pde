/*
/ ___| / ___| | | | ____|  \/  |  / \|_   _|  _ \|_ _\ \/ /
\___ \| |   | |_| |  _| | |\/| | / _ \ | | | |_) || | \  / 
 ___) | |___|  _  | |___| |  | |/ ___ \| | |  _ < | | /  \ 
|____/ \____|_| |_|_____|_|  |_/_/   \_\_| |_| \_\___/_/\_\
*/

//REVISIONS MADE BY SCGEMATRIX

/*Improved documentation and top-level comments for clarity and technical accuracy.
* Organized code into logical functions (`setup()`, `draw()`, `serialEvent()`, `drawAirplane()`, `drawCylinder()`).
* Renamed variables for clarity (`port` → `serialPort`, `interval` → `lastDataTime`), and removed unused arrays (`gravity`, `euler`, `ypr`).
* Simplified repetitive geometry generation in `drawCylinder()` using loop-based angle calculations.
* Reformatted packet parsing and synchronization logic in `serialEvent()` for better readability.
* Applied consistent indentation, alignment, and style across all code blocks.
* Added clear color annotations and structured shape definitions in `drawAirplane()`.
*/

/*
    MPU6050 Airplane Visualization (Processing + ToxicLibs)

    This Processing sketch visualizes a 3D airplane model that follows
    MPU6050 sensor orientation data via the InvenSense "Teapot" protocol.

    REQUIREMENTS:
    - ToxicLibs library:
      1. Download from: https://github.com/postspectacular/toxiclibs/releases
      2. Extract to: [userdir]/Documents/Processing/libraries
      3. Restart Processing

    - Correct Serial Port:
      Define the correct port in `setup()` (Serial.list()).
      Make sure no other application is using the same port.

    - Arduino side should send "OUTPUT_TEAPOT" formatted data.
*/

import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

ToxiclibsSupport gfx;
Serial serialPort;

// Teapot protocol packet
char[] teapotPacket = new char[14];
int serialCount = 0;
boolean synced = false;
int lastDataTime = 0;

// Quaternion & orientation
float[] q = new float[4];
Quaternion quat = new Quaternion(1, 0, 0, 0);

void setup() {
    size(1000, 1000, OPENGL);
    gfx = new ToxiclibsSupport(this);

    // Lighting & anti-aliasing
    lights();
    smooth();

    // Debug: List available ports
    // println(Serial.list());

    // Set correct COM port index
    serialPort = new Serial(this, Serial.list()[2], 9600);

    // Trigger DMP init/start
    serialPort.write('r');
}

void draw() {
    // Resend trigger if no data for >1s
    if (millis() - lastDataTime > 1000) {
        serialPort.write('r');
        lastDataTime = millis();
    }

    background(0);
    pushMatrix();
    translate(width / 2, height / 2);

    // Apply quaternion rotation
    float[] axis = quat.toAxisAngle();
    rotate(axis[0], -axis[1], axis[3], axis[2]);

    drawAirplane();
    popMatrix();
}

void serialEvent(Serial port) {
    lastDataTime = millis();

    while (port.available() > 0) {
        int incomingByte = port.read();

        // Initial sync on '$'
        if (!synced && incomingByte != '$') return;
        synced = true;

        // Packet validation
        if ((serialCount == 1 && incomingByte != 2) ||
            (serialCount == 12 && incomingByte != '\r') ||
            (serialCount == 13 && incomingByte != '\n')) {
            serialCount = 0;
            synced = false;
            return;
        }

        // Store packet data
        if (serialCount > 0 || incomingByte == '$') {
            teapotPacket[serialCount++] = (char) incomingByte;

            if (serialCount == 14) {
                serialCount = 0;

                // Parse quaternion data
                q[0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
                q[1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
                q[2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
                q[3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;

                // Adjust range if necessary
                for (int i = 0; i < 4; i++) {
                    if (q[i] >= 2) q[i] = -4 + q[i];
                }

                quat.set(q[0], q[1], q[2], q[3]);
            }
        }
    }
}

void drawAirplane() {
    // Main body (red)
    fill(255, 0, 0, 200);
    box(10, 10, 200);

    // Nose (blue)
    fill(0, 0, 255, 200);
    pushMatrix();
    translate(0, 0, -120);
    rotateX(PI / 2);
    drawCylinder(0, 20, 20, 8);
    popMatrix();

    // Wings & tail (green)
    fill(0, 255, 0, 200);
    beginShape(TRIANGLES);
    vertex(-100,  2,  30); vertex(0,  2, -80); vertex(100,  2,  30);
    vertex(-100, -2,  30); vertex(0, -2, -80); vertex(100, -2,  30);
    vertex( -2,   0,  98); vertex(-2, -30,  98); vertex(-2,  0,  70);
    vertex(  2,   0,  98); vertex( 2, -30,  98); vertex( 2,  0,  70);
    endShape();

    beginShape(QUADS);
    vertex(-100,  2, 30); vertex(-100, -2, 30); vertex(  0, -2, -80); vertex(  0,  2, -80);
    vertex( 100,  2, 30); vertex( 100, -2, 30); vertex(  0, -2, -80); vertex(  0,  2, -80);
    vertex(-100,  2, 30); vertex(-100, -2, 30); vertex(100, -2,  30); vertex(100,  2,  30);
    vertex(  -2,  0, 98); vertex(  2,  0, 98); vertex(  2, -30, 98); vertex( -2, -30, 98);
    vertex(  -2,  0, 98); vertex(  2,  0, 98); vertex(  2,   0, 70); vertex( -2,   0, 70);
    vertex(  -2, -30, 98); vertex(  2, -30, 98); vertex(  2,   0, 70); vertex( -2,   0, 70);
    endShape();
}

void drawCylinder(float topRadius, float bottomRadius, float height, int sides) {
    float angleStep = TWO_PI / sides;

    // Side surface
    beginShape(QUAD_STRIP);
    for (int i = 0; i <= sides; i++) {
        float angle = i * angleStep;
        vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
        vertex(bottomRadius * cos(angle), height, bottomRadius * sin(angle));
    }
    endShape();

    // Top cap
    if (topRadius != 0) {
        beginShape(TRIANGLE_FAN);
        vertex(0, 0, 0);
        for (int i = 0; i <= sides; i++) {
            float angle = i * angleStep;
            vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
        }
        endShape();
    }

    // Bottom cap
    if (bottomRadius != 0) {
        beginShape(TRIANGLE_FAN);
        vertex(0, height, 0);
        for (int i = 0; i <= sides; i++) {
            float angle = i * angleStep;
            vertex(bottomRadius * cos(angle), height, bottomRadius * sin(angle));
        }
        endShape();
    }
}
