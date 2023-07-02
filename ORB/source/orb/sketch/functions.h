//
//
int bin2dec(char s[]) {
// Convert binary string into decimal
int value = 0;
for (int i=0; i< strlen(s); i++)  // for every character in the string  strlen(s) returns the length of a char array
{
  value *= 2; // double the result so far
  if (s[i] == '1') value++;  //add 1 if needed
}
return value;
}
//
//


//
//
String padleft(String input, int length, String pad) {
// Pad string to left with supplied character
String output = input;
while (output.length() < length) {
  output = pad + output;
}
return output;
}
//
//

//
//
int chipselectBC(int chipSelectLines[] ){
// Select a Memory chip (1-4) to be used.
int chip = 0;
String binaryChip = "";
// Read chip select input pins
if (analogRead(A6) == 0) {   // read CS1 input pin
  binaryChip = "0";
}
else
{
  binaryChip = "1";
}
if (analogRead(A7) == 0) {   // read CS2 input pin
  binaryChip = binaryChip + "0";
}
else
{
    binaryChip = binaryChip + "1";
}
// 1 - 11
// 2 - 10
// 3 - 01
// 4 - 00
  if (binaryChip == "11")
    chip = 1;
  if (binaryChip == "10")
    chip = 2;
  if (binaryChip == "01")
    chip = 3;
  if (binaryChip == "00")
    chip = 4;
return chip;
}
//
//