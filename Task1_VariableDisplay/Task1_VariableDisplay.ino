// Variable to display
int N = 1;

void setup(void) {
  // Initialize serial communication
  Serial.begin(9600, SERIAL_8N1);
}

void loop(void) {
	Serial.print("Group Testing, ");
  // Display the value of N
  Serial.print("N = ");
  Serial.println(N);
	Serial.print('*');
// Delay for readability
  delay(3000);
}
