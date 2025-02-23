// PID Constants for each axis
#define Kp_X 2.0f // Proportional gain for X-axis
#define Ki_X 0.1f // Integral gain for X-axis
#define Kd_X 0.05f // Derivative gain for X-axis

#define Kp_Y 2.0f // Proportional gain for Y-axis
#define Ki_Y 0.1f // Integral gain for Y-axis
#define Kd_Y 0.05f // Derivative gain for Y-axis

// Sampling time (in seconds)
#define dt 0.001f // 1 ms sampling time (1 kHz control loop)

// Maximum control output (maps to PWM range 0-255)
#define Max_Control 255.0f

// Setpoint positions
const float Setpoint_X = 0.0f; // Target position for X-axis
const float Setpoint_Y = 0.0f; // Target position for Y-axis

// PID variables for X and Y axes
float Error_X = 0.0f, Previous_Error_X = 0.0f, Integral_X = 0.0f;
float Error_Y = 0.0f, Previous_Error_Y = 0.0f, Integral_Y = 0.0f;

// Actual positions from sensors
float Actual_Position_X = 0.0f;
float Actual_Position_Y = 0.0f;

// ADC values from sensors
int ADC_Value_X = 0;
int ADC_Value_Y = 0;

// Function to map ADC readings to physical positions
float Map_ADC_to_Position(int adc_value) {
  float Max_Position = 5.0f; // ±5 mm range
  return ((float)adc_value / 1023.0f) * (2.0f * Max_Position) - Max_Position;
}

void setup() {
  // Initialize PWM pins as output
  pinMode(3, OUTPUT);  // HIN for Coil 1
  pinMode(5, OUTPUT);  // LIN for Coil 1
  pinMode(6, OUTPUT);  // HIN for Coil 2
  pinMode(9, OUTPUT);  // LIN for Coil 2
  pinMode(10, OUTPUT); // HIN for Coil 3
  pinMode(11, OUTPUT); // LIN for Coil 3
  pinMode(12, OUTPUT); // HIN for Coil 4
  pinMode(13, OUTPUT); // LIN for Coil 4

  // Set analog input pins for ADC
  pinMode(A0, INPUT); // ADC for X-axis sensor
  pinMode(A1, INPUT); // ADC for Y-axis sensor

  // Start serial monitor for debugging
  Serial.begin(9600);
}

void loop() {
  Control_Loop();
  delay(1); // 1 ms delay for 1 kHz loop frequency
}

void Control_Loop() {
  // 1. Read ADC values
  ADC_Value_X = analogRead(A0);
  ADC_Value_Y = analogRead(A1);

  // 2. Map ADC values to physical positions
  Actual_Position_X = Map_ADC_to_Position(ADC_Value_X);
  Actual_Position_Y = Map_ADC_to_Position(ADC_Value_Y);

  // 3. Calculate errors
  Error_X = Setpoint_X - Actual_Position_X;
  Error_Y = Setpoint_Y - Actual_Position_Y;

  // 4. Calculate PID terms for X-axis
  Integral_X += Error_X * dt;
  float Derivative_X = (Error_X - Previous_Error_X) / dt;
  float Control_Output_X = Kp_X * Error_X + Ki_X * Integral_X + Kd_X * Derivative_X;
  Previous_Error_X = Error_X;

  // 5. Calculate PID terms for Y-axis
  Integral_Y += Error_Y * dt;
  float Derivative_Y = (Error_Y - Previous_Error_Y) / dt;
  float Control_Output_Y = Kp_Y * Error_Y + Ki_Y * Integral_Y + Kd_Y * Derivative_Y;
  Previous_Error_Y = Error_Y;

  // 6. Map control outputs to PWM duty cycles
  int PWM_Coil1 = max(0, min(255, (int)((Control_Output_X / Max_Control) * 255)));
  int PWM_Coil2 = max(0, min(255, (int)((-Control_Output_X / Max_Control) * 255)));
  int PWM_Coil3 = max(0, min(255, (int)((Control_Output_Y / Max_Control) * 255)));
  int PWM_Coil4 = max(0, min(255, (int)((-Control_Output_Y / Max_Control) * 255)));

  // 7. Write PWM signals to IR2110 drivers
  analogWrite(3, PWM_Coil1);  // HIN for Coil 1
  analogWrite(5, PWM_Coil2);  // LIN for Coil 1
  analogWrite(6, PWM_Coil3);  // HIN for Coil 2
  analogWrite(9, PWM_Coil4);  // LIN for Coil 2
  analogWrite(10, PWM_Coil1); // HIN for Coil 3
  analogWrite(11, PWM_Coil2); // LIN for Coil 3
  analogWrite(12, PWM_Coil3); // HIN for Coil 4
  analogWrite(13, PWM_Coil4); // LIN for Coil 4

  // 8. Debugging output
  Serial.print("X Position: "); Serial.print(Actual_Position_X);
  Serial.print(", Y Position: "); Serial.println(Actual_Position_Y);
  Serial.print("Control Output X: "); Serial.print(Control_Output_X);
  Serial.print(", Control Output Y: "); Serial.println(Control_Output_Y);
}
