/*Assume that you have entered 34,13.75,AUST in the InputBox of Serial Monitor
with Line ending option at Newline. Create sketch to receive the string and
isolate the language elements (he tokens) using strtok() function and save them
into appropriate variables.*/

char arr[20];
char *token;
void setup(){
Serial.begin(9600);
}
void loop(){
 if(Serial.available()!=0){
 byte m = Serial.readBytesUntil('\n', arr, 20);
 arr[m] = '\0';
 token = strtok(arr,",");
 byte y1 = atof(token);
 Serial.println(y1, DEC); //shows: 34
 token = strtok(NULL,",");
 float y2 = atof(token);
 Serial.println(y2, 2); //shows: 13.75
 token = strtok(NULL,",");
 Serial.println(token); //shows: AUST
 }
}
