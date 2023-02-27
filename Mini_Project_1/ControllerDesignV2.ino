// PID Controller Constants
double Kp = 8.05114690297919;  // Proportional gain
double Ki = 0.716404922216221;  // Integral gain
double Kd = 0;  // Derivative gain

// Variables for PID controller
double setpoint = 0;  // Desired position/setpoint
double input, //actual position
double output; //angular motor position
double error, last_error;
double integral, derivative;
double I = 0;
double e_past = 0;
double Tc = 0;
double Ts = 0; //elapsed time


// Other variables
unsigned long last_time;
double sample_time = 100;  // Sample time in milliseconds
double out_min = 0;
double out_max = 255;

void setup() {
  // Setup pin modes and other initializations
}

void loop() {
  // Calculate time since last sample
  unsigned long now = millis();
  double elapsed_time = (double)(now - last_time);

  // Read output: Read y, current value of system output
  input = encoder.read();

  // Calculate error: desired position minus actual position
  error = setpoint - input;

if(Ts > 0){
  // Calculate derivative terms
  derivative = (error - last_error) / elapsed_time;

  // Save current error for next loop
  last_error = error;

} else{
  derivative = 0;
}

  // Calculate integral terms
  integral += (error * elapsed_time);

  // Calculate controller output
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Constrain output within minimum and maximum values
  output = constrain(output, out_min, out_max);

  // Output to the system
  analogWrite(3, output);

  // Save current time for next loop
  last_time = now;

  // Wait for next sample time
  delay(sample_time);
}
