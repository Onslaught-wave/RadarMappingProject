import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;

Serial myPort;
String angle = "";
String data = "";
float pixsDistance1, pixsDistance2, pixsDistance3;
int iAngle, iDistance1, iDistance2, iDistance3;
int scanDirection = 1; 

// Arrays to store point history
int maxPoints = 5000;
float[] pointX = new float[maxPoints];
float[] pointY = new float[maxPoints];
int[] pointColors = new int[maxPoints];
int[] pointLifetime = new int[maxPoints];
int currentPoint = 0;
float pointFadeTime = 1000;

// Sensor calibration factors - significantly adjusted for HC-SR04
float hcsr04CalibrationFactor = 1.25;  // Increased from 1.35 to 1.85 to match other sensors
float tfminiCalibrationFactor = 1.0;
float vl53l0xCalibrationFactor = 0.95;

// Sensor offsets in mm to account for their different positions
int hcsr04Offset = -40; // Removed the negative offset
int tfminiOffset = 0;
int vl53l0xOffset = 0;


float forwardAngleCorrection = 0.0; 
float reverseAngleCorrection = 2.5; 

// Arrays for advanced filtering for HC-SR04
float[] lastDistances1 = new float[12]; // Increased to 12 samples for better filtering
int filterIndex = 0;

// Minimum reliable detection distances
int hcsr04MinDistance = 40; // HC-SR04 minimum reliable distance in mm

// Maximum detection distance for HC-SR04 (more restrictive to avoid false readings)
int hcsr04MaxDistance = 800; // Max reliable distance in mm for HC-SR04

// Flag to enable cross-sensor validation
boolean enableCrossSensorValidation = true;

void setup() {
  size(1200, 600);
  smooth();
  println(Serial.list());
  String portName = "/dev/cu.usbserial-1420"; // Your port
  println("Using " + portName);
  myPort = new Serial(this, portName, 115200);
  myPort.bufferUntil('\n');
  
  // Initialize point arrays
  for (int i = 0; i < maxPoints; i++) {
    pointX[i] = 0;
    pointY[i] = 0;
    pointColors[i] = 0;
    pointLifetime[i] = 0;
  }
  
  // Initialize filter arrays
  for (int i = 0; i < lastDistances1.length; i++) {
    lastDistances1[i] = 0;
  }
  
  // Set background once to black
  background(0);
}

void draw() {
  // Semi-transparent background for trail effect
  fill(0, 15);
  rect(0, 0, width, height);
  
  // Draw the radar components
  drawRadar();
  drawObjects();
  drawText();
  
  
}

void serialEvent(Serial myPort) {

  delay(150);
  
  data = myPort.readStringUntil('\n');
  if (data != null && data.indexOf("data") != -1) {
    String[] parts = data.split(",");
    if (parts.length >= 5) { 
      angle = parts[1];
      float rawAngle = float(angle);
      
     
      if (parts.length >= 6) {
        scanDirection = int(parts[5]);
      }
      
      
      float correctedAngle = rawAngle;
      if (scanDirection > 0) {
        correctedAngle += forwardAngleCorrection;
      } else {
        correctedAngle += reverseAngleCorrection;
      }
      
      iAngle = int(correctedAngle);
      
      // Get raw distance values from sensors
      float rawDistance1 = float(parts[2]);
      float rawDistance2 = float(parts[3]);
      float rawDistance3 = float(parts[4]);
      
      // First check if HC-SR04 value is valid
      if (rawDistance1 < 10 || rawDistance1 > 4000) {
        rawDistance1 = 0; // Invalidate reading if it's outside reasonable bounds
      }
      
      // Apply calibration factors and offsets
      float calibratedDistance1 = rawDistance1 * hcsr04CalibrationFactor + hcsr04Offset;
      int calibratedDistance2 = int(rawDistance2 * tfminiCalibrationFactor + tfminiOffset);
      int calibratedDistance3 = int(rawDistance3 * vl53l0xCalibrationFactor + vl53l0xOffset);
      
      // Apply HC-SR04 specific calibration - exponential correction for better accuracy
      calibratedDistance1 = improvedHCSR04Calibration(calibratedDistance1);
      
      // Store HC-SR04 value in circular buffer for median filtering
      lastDistances1[filterIndex] = calibratedDistance1;
      filterIndex = (filterIndex + 1) % lastDistances1.length;
      
      // Calculate median value for HC-SR04
      float filteredDistance1 = medianFilter(lastDistances1);
      
      // Remove outliers by comparing with recent history
      if (filteredDistance1 > 0) {
        filteredDistance1 = removeOutliers(filteredDistance1, lastDistances1);
      }
      
      // Cross-sensor validation and correction
      if (enableCrossSensorValidation && rawDistance2 > 0 && rawDistance3 > 0) {
        // Average of other sensors (which are generally more reliable)
        float referenceDistance = (calibratedDistance2 + calibratedDistance3) / 2.0;
        
        // If HC-SR04 differs significantly from other sensors, adjust it
        if (abs(filteredDistance1 - referenceDistance) > (referenceDistance * 0.2)) {
          // Weight HC-SR04 less and reference more
          filteredDistance1 = filteredDistance1 * 0.2 + referenceDistance * 0.8;
        }
      }
      
      // Constrain distances to valid ranges
      iDistance1 = constrain(int(filteredDistance1), hcsr04MinDistance, hcsr04MaxDistance);
      iDistance2 = constrain(calibratedDistance2, 50, 1000);
      iDistance3 = constrain(calibratedDistance3, 50, 1000);

      // Set scale factor consistently
      float scaleFactor = (min(width, height) * 0.8) / 1000.0;
      
      // Calculate pixel distances with proper scaling
      pixsDistance1 = iDistance1 * scaleFactor;
      pixsDistance2 = iDistance2 * scaleFactor;
      pixsDistance3 = iDistance3 * scaleFactor;
      
      // Only add HC-SR04 point if value is valid and not likely to be an error
      if (iDistance1 > hcsr04MinDistance && iDistance1 < hcsr04MaxDistance) {
        addPoint(iAngle, pixsDistance1, color(255, 0, 0));     // HC-SR04
      }
      
      if (iDistance2 > 0) addPoint(iAngle, pixsDistance2, color(0, 255, 0));     // TFmini
      if (iDistance3 > 0) addPoint(iAngle, pixsDistance3, color(255, 255, 255)); // VL53L0X
      
      // Print distances in cm for debugging
      System.out.printf("Angle: %d°, Direction: %d, Distance: %.1f cm, %.1f cm, %.1f cm\n", 
                        iAngle, scanDirection, iDistance1/10.0, iDistance2/10.0, iDistance3/10.0);
    }
  }
}

float improvedHCSR04Calibration(float distance) {
  // Progressive calibration for HC-SR04 that better matches real world behavior
  if (distance < 200) {
    return distance * 1.5; // Significant boost for close range
  } else if (distance < 400) {
    return 200 * 1.5 + (distance - 200) * 1.3; // Smaller boost for mid-range
  } else if (distance < 600) {
    return 200 * 1.5 + 200 * 1.3 + (distance - 400) * 1.2; // Even smaller boost
  } else {
    return 200 * 1.5 + 200 * 1.3 + 200 * 1.2 + (distance - 600) * 1.1; // Minimal boost for far range
  }
}

float removeOutliers(float current, float[] history) {
  // Calculate standard deviation and mean of valid values
  float sum = 0;
  int count = 0;
  
  for (int i = 0; i < history.length; i++) {
    if (history[i] > 0) {
      sum += history[i];
      count++;
    }
  }
  
  if (count < 3) return current; // Not enough data points
  
  float mean = sum / count;
  float sumSq = 0;
  
  for (int i = 0; i < history.length; i++) {
    if (history[i] > 0) {
      sumSq += (history[i] - mean) * (history[i] - mean);
    }
  }
  
  float stdDev = sqrt(sumSq / count);
  
  // If current value is too far from mean (more than 2 std devs), adjust it
  if (abs(current - mean) > 2 * stdDev) {
    return mean;
  }
  
  return current;
}

float medianFilter(float[] values) {
  // Create a copy of valid values
  ArrayList<Float> validValues = new ArrayList<Float>();
  
  for (int i = 0; i < values.length; i++) {
    if (values[i] > 0) {
      validValues.add(values[i]);
    }
  }
  
  if (validValues.size() == 0) return 0;
  
  // Convert ArrayList to array for sorting
  float[] validArray = new float[validValues.size()];
  for (int i = 0; i < validValues.size(); i++) {
    validArray[i] = validValues.get(i);
  }
  
  // Sort array
  java.util.Arrays.sort(validArray);
  
  // Return median
  if (validArray.length % 2 == 0) {
    return (validArray[validArray.length/2] + validArray[validArray.length/2 - 1]) / 2;
  } else {
    return validArray[validArray.length/2];
  }
}

void addPoint(int angle, float distance, color col) {
  // Calculate cartesian coordinates
  float radAngle = radians(angle);
  float x = distance * cos(radAngle);
  float y = -distance * sin(radAngle);
  
  
  boolean overlapped = false;
  float overlapThreshold = 5; // Points within this many pixels will be considered overlapping
  
  for (int i = 0; i < maxPoints; i++) {
    if (pointLifetime[i] > 0) {
      float dx = pointX[i] - x;
      float dy = pointY[i] - y;
      float distance2 = sqrt(dx*dx + dy*dy);      
      if (distance2 < overlapThreshold) {
        // Обновляем точку с более высоким приоритетом на основе цвета и времени жизни
        pointColors[i] = col; // Last sensor to detect gets priority
        pointLifetime[i] = (int)pointFadeTime; // Обновляем время жизни
        overlapped = true;
        break;
      }
    }
  }
  
  // If no overlay found, add as new point
  if (!overlapped) {
    pointX[currentPoint] = x;
    pointY[currentPoint] = y;
    pointColors[currentPoint] = col;
    pointLifetime[currentPoint] = (int)pointFadeTime;
    currentPoint = (currentPoint + 1) % maxPoints;
  }
}

void drawRadar() {
  pushMatrix();
  translate(width / 2, height - height * 0.1);
  noFill();
  strokeWeight(2);
  stroke(98, 245, 31);
  
  float scaleFactor = (min(width, height) * 0.8) / 1000.0;

  // Draw arcs every 200 mm (20 cm) up to 1000 mm (100 cm)
  for (int r = 200; r <= 1000; r += 200) {
    float radius = r * scaleFactor;
    arc(0, 0, radius * 2, radius * 2, PI, TWO_PI);
  }

  // Draw angle lines every 30°
  for (int i = 0; i <= 180; i += 30) {
    float maxRadius = 1000 * scaleFactor;
    line(0, 0, maxRadius * cos(radians(i)), -maxRadius * sin(radians(i)));
  }
  popMatrix();
}

void drawObjects() {
  pushMatrix();
  translate(width / 2, height - height * 0.1);
  
  // Draw all points with fading based on lifetime
  for (int i = 0; i < maxPoints; i++) {
    if (pointLifetime[i] > 0) {
      // Calculate alpha based on lifetime
      int alpha = 255;
      if (pointLifetime[i] < 255) {
        alpha = (int)pointLifetime[i];
      }
      
      // Draw point
      color pointColor = pointColors[i];
      stroke(red(pointColor), green(pointColor), blue(pointColor), alpha);
      strokeWeight(5);
      point(pointX[i], pointY[i]);
    }
  }
  
  popMatrix();
}

void drawText() {
  pushMatrix();
  fill(98, 245, 31);
  textSize(22);

  float scaleFactor = (min(width, height) * 0.8) / 1000.0;
  int baseY = height - 30;

  // Distance markers every 20 cm (200 mm)
  for (int i = 1; i <= 5; i++) {
    int distCm = i * 20;
    float x = width/2 + (i * 200 * scaleFactor * cos(radians(0)));
    text(distCm + "cm", x - 20, baseY);
  }

  // Sensor data display
  textSize(20);
  int infoX = 20;
  int infoY = 30;

  fill(255, 0, 0);
  text("HC-SR04: " + (iDistance1/10) + " cm", infoX, infoY);

  fill(0, 255, 0);
  text("TFmini: " + (iDistance2/10) + " cm", infoX, infoY + 30);

  fill(255, 255, 255);
  text("VL53L0X: " + (iDistance3/10) + " cm", infoX, infoY + 60);

  fill(98, 245, 31);
  text("Angle: " + iAngle + "°", infoX, infoY + 90);
  text("Direction: " + (scanDirection > 0 ? "Forward" : "Reverse"), infoX, infoY + 120);
  text("Press 'S' to save data", infoX, infoY + 150);
  text("Press 'C' to clear points", infoX, infoY + 180);
  
  popMatrix();
}

void keyPressed() {
  // HCSR04 calibration
  if (key == '1') hcsr04CalibrationFactor += 0.05;
  if (key == 'q') hcsr04CalibrationFactor -= 0.05;
  
  // TFMini calibration
  if (key == '2') tfminiCalibrationFactor += 0.01;
  if (key == 'w') tfminiCalibrationFactor -= 0.01;
  
  // VL53L0X calibration
  if (key == '3') vl53l0xCalibrationFactor += 0.01;
  if (key == 'e') vl53l0xCalibrationFactor -= 0.01;
  
  
  if (key == '4') forwardAngleCorrection += 0.5;
  if (key == 'r') forwardAngleCorrection -= 0.5;
  if (key == '5') reverseAngleCorrection += 0.5;
  if (key == 't') reverseAngleCorrection -= 0.5;
  
  // Reset all calibrations
  if (key == '0') {
    hcsr04CalibrationFactor = 1.30;
    tfminiCalibrationFactor = 1.0;
    vl53l0xCalibrationFactor = 0.95;
    forwardAngleCorrection = 0.0;
    reverseAngleCorrection = 2.5;
  }
  
  // Offsets
  if (key == 'a') hcsr04Offset += 10;
  if (key == 'z') hcsr04Offset -= 10;
  if (key == 's') tfminiOffset += 10;
  if (key == 'x') tfminiOffset -= 10;
  if (key == 'd') vl53l0xOffset += 10;
  if (key == 'c') vl53l0xOffset -= 10;
  
  // Toggle cross-sensor validation
  if (key == 'v') {
    enableCrossSensorValidation = !enableCrossSensorValidation;
    println("Cross-sensor validation: " + (enableCrossSensorValidation ? "enabled" : "disabled"));
  }
  
  // Save data
  if (key == 'S') {
    saveData();
  }
  
  // Clear all points
  if (key == 'C') {
    clearPoints();
  }
  
  println("Calibration: HCSR04=" + hcsr04CalibrationFactor + 
          ", TFMini=" + tfminiCalibrationFactor + 
          ", VL53L0X=" + vl53l0xCalibrationFactor);
  println("Angle correction: Forward=" + forwardAngleCorrection +
          "°, Reverse=" + reverseAngleCorrection + "°");
  println("Offsets: HCSR04=" + hcsr04Offset + 
          "mm, TFMini=" + tfminiOffset + 
          "mm, VL53L0X=" + vl53l0xOffset + "mm");
}

void clearPoints() {
  for (int i = 0; i < maxPoints; i++) {
    pointLifetime[i] = 0;
    pointX[i] = 0;
    pointY[i] = 0;
    pointColors[i] = 0;
  }
  println("All points cleared!");
}

void saveData() {
  // Save to Documents/Processing folder
  String filePath = System.getProperty("user.home") + "/Documents/Processing/";
  String fileName = "radar_data_" + year() + month() + day() + hour() + minute() + ".csv";
  
  // Create directory if it doesn't exist
  File directory = new File(filePath);
  if (!directory.exists()) {
    directory.mkdirs();
    println("Created directory: " + filePath);
  }
  
  PrintWriter output = createWriter(filePath + fileName);
  
  // Write header with semicolons (for Excel with Russian locale)
  output.println("Angle;Direction;HC-SR04(cm);TFmini(cm);VL53L0X(cm);X;Y");
  
  // We need to gather data based on points that exist
  for (int i = 0; i < maxPoints; i++) {
    if (pointLifetime[i] > 0) {
      // Get sensor type from point color
      color pointColor = pointColors[i];
      
      // Convert back from screen coordinates to real values
      float distance = sqrt(pointX[i]*pointX[i] + pointY[i]*pointY[i]);
      float scaleFactor = (min(width, height) * 0.8) / 1000.0;
      distance = distance / scaleFactor; // Convert back to mm
      
      // Calculate angle from point coordinates
      float angleRad = atan2(-pointY[i], pointX[i]);
      float angleDeg = degrees(angleRad);
      
      // Write data row with semicolons and replace decimal points with commas
      String line = nf(angleDeg, 0, 1).replace(".", ",") + ";" +
                   (angleRad < HALF_PI ? "1" : "-1") + ";" +
                   (red(pointColor) > 0 ? nf(distance/10.0, 0, 1).replace(".", ",") : "0") + ";" +
                   (green(pointColor) > 0 ? nf(distance/10.0, 0, 1).replace(".", ",") : "0") + ";" +
                   (blue(pointColor) > 0 && red(pointColor) > 0 && green(pointColor) > 0 ? 
                   nf(distance/10.0, 0, 1).replace(".", ",") : "0") + ";" +
                   nf(pointX[i], 0, 1).replace(".", ",") + ";" +
                   nf(pointY[i], 0, 1).replace(".", ",");
      
      output.println(line);
    }
  }
  
  output.flush();
  output.close();
  println("Data saved to " + filePath + fileName);
}
