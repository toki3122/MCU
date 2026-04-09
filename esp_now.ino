#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <math.h>

#define SG90_L_PIN       18
#define SG90_R_PIN       19
#define MG996_FRONT_PIN  21
#define MG996_BACK_PIN   22
#define HALL_LEFT_PIN    34
#define HALL_CENTER_PIN  35
#define HALL_RIGHT_PIN   32
#define EM_PIN           27
#define BAT_PIN          33
#define MPU_ADDR         0x68

#define WALK_FWD         94
#define WALK_REV         86
#define STOP_SPD         90
#define CREEP            91

#define HALL_DETECT      800
#define HALL_CLOSE       2000
#define HALL_CONTACT     3000
#define HALL_LOCK        3500
#define ALIGN_TOL        80

#define RSSI_HELP       -75
#define RSSI_NEAR       -50

#define LEVY_ALPHA       1.5f
#define LEVY_MIN         0.2f
#define LEVY_MAX         3.0f
#define SPEED_MPS        0.05f
#define CF_TAU           0.3f

#define TILT_SOS         30.0f
#define STUCK_THR        0.05f
#define HEARTBEAT_MS     200

Servo sg90L, sg90R, mg996F, mg996B;

float AccX, AccY, AccZ;
float RateRoll, RatePitch, RateYaw;
float AngleRoll=0, AnglePitch=0;
float AngleRollAcc, AnglePitchAcc;
float GCR=0, GCP=0, GCY=0;
float TempC;
unsigned long imuPrev=0;
bool imu_stuck=false;
float currentHeading=0, targetHeading=0;
float pos_x=0, pos_y=0;
float levyStep=0;
unsigned long lastLoop=0, lastHB=0;

uint8_t myMAC[6];
uint8_t bcastAddr[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

bool isMerged=false;
bool isSOS=false;
bool isHelper=false;
uint8_t partnerMAC[6]={0};
int8_t bestHelperRSSI=-127;

typedef struct __attribute__((packed)) {
    uint8_t  type;
    uint8_t  senderID[6];
    float    pos_x, pos_y, heading;
    int8_t   rssi;
    uint8_t  battery;
    bool     sosFlag;
    float    roll, pitch;
    int16_t  hallC;
} Pkt;

#define PKT_DATA     0
#define PKT_SOS      1
#define PKT_SOS_ACK  2
#define PKT_COMMIT   3
#define PKT_DOCK_RDY 4
#define PKT_LOCKED   5
#define PKT_DISENGAGE 6

uint16_t seq=0;

void emOn()  { digitalWrite(EM_PIN,HIGH); }
void emOff() { digitalWrite(EM_PIN,LOW);  }

void allStop() {
    if(sg90L.attached())  sg90L.write(STOP_SPD);
    if(sg90R.attached())  sg90R.write(STOP_SPD);
}

void driveFwd() {
    if(!sg90L.attached()) sg90L.attach(SG90_L_PIN);
    if(!sg90R.attached()) sg90R.attach(SG90_R_PIN);
    sg90L.write(WALK_FWD); sg90R.write(WALK_FWD);
}

void driveRev(int ms) {
    if(!sg90L.attached()) sg90L.attach(SG90_L_PIN);
    if(!sg90R.attached()) sg90R.attach(SG90_R_PIN);
    sg90L.write(WALK_REV); sg90R.write(WALK_REV);
    if(ms>0){delay(ms);allStop();}
}

void imu_init() {
    Wire.begin(); Wire.setClock(400000);
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
    delay(200);
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();
}

void imu_read_raw() {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission();
    Wire.requestFrom(MPU_ADDR,14);
    int16_t AX=Wire.read()<<8|Wire.read();
    int16_t AY=Wire.read()<<8|Wire.read();
    int16_t AZ=Wire.read()<<8|Wire.read();
    int16_t TM=Wire.read()<<8|Wire.read();
    int16_t GX=Wire.read()<<8|Wire.read();
    int16_t GY=Wire.read()<<8|Wire.read();
    int16_t GZ=Wire.read()<<8|Wire.read();
    AccX=AX/4096.0f; AccY=AY/4096.0f; AccZ=AZ/4096.0f;
    TempC=(TM/340.0f)+36.53f;
    RateRoll=(GX/65.5f)-GCR; RatePitch=(GY/65.5f)-GCP; RateYaw=(GZ/65.5f)-GCY;
}

void imu_calibrate() {
    Serial.println("IMU calibrating — keep still...");
    GCR=0; GCP=0; GCY=0;
    for(int i=0;i<2000;i++){imu_read_raw();GCR+=RateRoll;GCP+=RatePitch;GCY+=RateYaw;delay(1);}
    GCR/=2000; GCP/=2000; GCY/=2000;
    imu_read_raw();
    AngleRoll =atan2f(AccY,sqrtf(AccX*AccX+AccZ*AccZ))*57.2958f;
    AnglePitch=atan2f(-AccX,sqrtf(AccY*AccY+AccZ*AccZ))*57.2958f;
    imuPrev=millis();
    Serial.println("IMU calibration done");
}

void imu_update(float dt) {
    imu_read_raw();
    AngleRollAcc =atan2f(AccY,sqrtf(AccX*AccX+AccZ*AccZ))*57.2958f;
    AnglePitchAcc=atan2f(-AccX,sqrtf(AccY*AccY+AccZ*AccZ))*57.2958f;
    float alpha=CF_TAU/(CF_TAU+dt);
    AngleRoll =alpha*(AngleRoll +RateRoll *dt)+(1.0f-alpha)*AngleRollAcc;
    AnglePitch=alpha*(AnglePitch+RatePitch*dt)+(1.0f-alpha)*AnglePitchAcc;
    imu_stuck=(fabsf(AccX)<STUCK_THR);
    currentHeading+=RateYaw*dt;
    if(currentHeading>=360.0f) currentHeading-=360.0f;
    if(currentHeading<0.0f)    currentHeading+=360.0f;
}

float getTilt() { return fmaxf(fabsf(AngleRoll),fabsf(AnglePitch)); }

float levySample() {
    float u=(float)random(1,10000)/10000.0f;
    return constrain(LEVY_MIN/powf(u,1.0f/LEVY_ALPHA),LEVY_MIN,LEVY_MAX);
}

float headingErr() {
    float e=targetHeading-currentHeading;
    while(e>180.0f)e-=360.0f; while(e<-180.0f)e+=360.0f; return e;
}

void updateDR(float dt) {
    float spd=SPEED_MPS;
    float rad=currentHeading*(float)M_PI/180.0f;
    pos_x+=spd*cosf(rad)*dt; pos_y+=spd*sinf(rad)*dt;
}

void sendPkt(uint8_t type, uint8_t* dest=nullptr) {
    Pkt p={};
    p.type=type; memcpy(p.senderID,myMAC,6);
    p.pos_x=pos_x; p.pos_y=pos_y; p.heading=currentHeading;
    p.battery=(uint8_t)constrain((int)map(analogRead(BAT_PIN),600,840,0,100),0,100);
    p.sosFlag=isSOS; p.roll=AngleRoll; p.pitch=AnglePitch;
    esp_now_send(dest?dest:bcastAddr,(uint8_t*)&p,sizeof(p));
}

void onSent(const uint8_t* mac, esp_now_send_status_t s){}

void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if(len<(int)sizeof(Pkt)) return;
    Pkt p; memcpy(&p,data,sizeof(Pkt));
    int8_t rssi=info->rx_ctrl->rssi;
    uint8_t* src=(uint8_t*)info->src_addr;

    if(p.type==PKT_SOS && !isMerged && !isSOS && rssi>RSSI_HELP) {
        if(rssi>bestHelperRSSI) { bestHelperRSSI=rssi; memcpy(partnerMAC,src,6); }
        Pkt ack={}; ack.type=PKT_SOS_ACK; memcpy(ack.senderID,myMAC,6);
        ack.rssi=rssi;
        esp_now_send(src,(uint8_t*)&ack,sizeof(ack));
    }

    if(p.type==PKT_SOS_ACK && isSOS) {
        if(rssi>bestHelperRSSI) { bestHelperRSSI=rssi; memcpy(partnerMAC,src,6); }
    }

    if(p.type==PKT_COMMIT && memcmp(p.senderID,partnerMAC,6)==0) {
        isHelper=true;
        allStop();
        mg996B.write(90);
    }

    if(p.type==PKT_DOCK_RDY && isHelper) {
        targetHeading=p.heading;
    }

    if(p.type==PKT_LOCKED && (isSOS||isHelper)) {
        isMerged=true; isSOS=false; isHelper=false;
        Serial.println("MERGED");
    }

    if(p.type==PKT_DISENGAGE && isMerged) {
        emOff(); isMerged=false; bestHelperRSSI=-127;
        memset(partnerMAC,0,6);
        levyStep=levySample();
        targetHeading=(float)random(0,360);
        Serial.println("DETACHED — resuming levy");
    }
}

void levyWalk(float dt) {
    if(getTilt()>TILT_SOS || imu_stuck) {
        allStop();
        isSOS=true; bestHelperRSSI=-127;
        Serial.println("SOS triggered");
        return;
    }
    float err=headingErr();
    int corr=constrain((int)(err/5),-6,6);
    if(!sg90L.attached()) sg90L.attach(SG90_L_PIN);
    if(!sg90R.attached()) sg90R.attach(SG90_R_PIN);
    sg90L.write(constrain(WALK_FWD+corr,82,98));
    sg90R.write(constrain(WALK_FWD-corr,82,98));
    levyStep-=SPEED_MPS*(60.0f/1000.0f);
    if(levyStep<=0) {
        allStop();
        levyStep=levySample();
        float turn=(random(0,100)<80)?(float)random(-45,45):(float)random(-180,180);
        targetHeading=fmodf(currentHeading+turn+360.0f,360.0f);
        delay(100);
    }
    updateDR(dt);
}

void sosBroadcast() {
    static unsigned long lastSOS=0;
    static int retryCnt=0;
    if(millis()-lastSOS>500) {
        sendPkt(PKT_SOS);
        lastSOS=millis(); retryCnt++;
        Serial.print("SOS #"); Serial.println(retryCnt);
    }
    if(bestHelperRSSI>RSSI_HELP && retryCnt>2) {
        Pkt c={}; c.type=PKT_COMMIT; memcpy(c.senderID,myMAC,6);
        esp_now_send(partnerMAC,(uint8_t*)&c,sizeof(c));
        mg996B.write(90);
        retryCnt=0;
        Serial.println("Helper committed — awaiting dock");
    }
}

void convergeToDest() {
    int hL=analogRead(HALL_LEFT_PIN);
    int hC=analogRead(HALL_CENTER_PIN);
    int hR=analogRead(HALL_RIGHT_PIN);
    int tot=hL+hC+hR;

    if(tot<HALL_DETECT) {
        if(!sg90L.attached()) sg90L.attach(SG90_L_PIN);
        if(!sg90R.attached()) sg90R.attach(SG90_R_PIN);
        float err=headingErr();
        int corr=constrain((int)(err/5),-6,6);
        sg90L.write(constrain(CREEP+corr,88,96));
        sg90R.write(constrain(CREEP-corr,88,96));
        delay(50); return;
    }

    Pkt rep={}; rep.type=PKT_DATA; memcpy(rep.senderID,myMAC,6); rep.hallC=(int16_t)hC;
    esp_now_send(bcastAddr,(uint8_t*)&rep,sizeof(rep));

    int err=hR-hL;
    int corr=err/4;
    if(!sg90L.attached()) sg90L.attach(SG90_L_PIN);
    if(!sg90R.attached()) sg90R.attach(SG90_R_PIN);
    sg90L.write(constrain(CREEP+corr,88,96));
    sg90R.write(constrain(CREEP-corr,84,92));

    if(abs(err)<ALIGN_TOL && hC>hL && hC>hR && tot>HALL_CLOSE) {
        allStop();

        int bestAngle=60, bestRead=0;
        for(int a=60;a<=120;a+=2) {
            mg996B.write(a); delay(40);
            int r=analogRead(HALL_CENTER_PIN);
            if(r>bestRead){bestRead=r;bestAngle=a;}
        }
        mg996B.write(bestAngle);
        sendPkt(PKT_DOCK_RDY);
        delay(200);

        while(analogRead(HALL_CENTER_PIN)<HALL_CONTACT) {
            if(!sg90L.attached()) sg90L.attach(SG90_L_PIN);
            if(!sg90R.attached()) sg90R.attach(SG90_R_PIN);
            sg90L.write(CREEP); sg90R.write(CREEP);
            delay(50); allStop(); delay(30);
        }
        sg90L.detach(); sg90R.detach();

        emOn(); delay(200);
        int r1=analogRead(HALL_CENTER_PIN); delay(50);
        int r2=analogRead(HALL_CENTER_PIN);

        if(abs(r1-r2)<50 && r1>HALL_LOCK) {
            sendPkt(PKT_LOCKED);
            isMerged=true; isHelper=false;
            Serial.println("LOCKED — MERGED");
        } else {
            emOff(); driveRev(150);
            sg90L.attach(SG90_L_PIN); sg90R.attach(SG90_R_PIN);
        }
    }
}

void mergedRun() {
    static unsigned long lastTick=0;
    if(millis()-lastTick<60) return;
    lastTick=millis();

    float err=headingErr();
    int corr=constrain((int)(err/5),-6,6);
    if(!sg90L.attached()) sg90L.attach(SG90_L_PIN);
    if(!sg90R.attached()) sg90R.attach(SG90_R_PIN);
    sg90L.write(constrain(WALK_FWD+corr,82,98));
    sg90R.write(constrain(WALK_FWD-corr,82,98));

    static float dist=0;
    dist+=SPEED_MPS*(60.0f/1000.0f);
    if(getTilt()<10.0f && dist>0.3f) {
        sendPkt(PKT_DISENGAGE);
        emOff(); isMerged=false; dist=0;
        bestHelperRSSI=-127; memset(partnerMAC,0,6);
        levyStep=levySample(); targetHeading=(float)random(0,360);
        Serial.println("DETACHED — resuming levy");
    }
}

void setup() {
    Serial.begin(115200);
    imu_init();

    sg90L.attach(SG90_L_PIN); sg90R.attach(SG90_R_PIN);
    mg996F.attach(MG996_FRONT_PIN); mg996B.attach(MG996_BACK_PIN);
    allStop(); mg996F.write(90); mg996B.write(90);
    pinMode(EM_PIN,OUTPUT); emOff();

    WiFi.mode(WIFI_STA); WiFi.disconnect();
    esp_wifi_set_channel(6,WIFI_SECOND_CHAN_NONE);
    esp_wifi_get_mac(WIFI_IF_STA,myMAC);
    randomSeed((uint32_t)myMAC[5]<<8|myMAC[4]);

    imu_calibrate();

    if(esp_now_init()!=ESP_OK){Serial.println("ESP-NOW fail");return;}
    esp_now_register_recv_cb(onRecv);
    esp_now_register_sent_cb(onSent);
    esp_now_peer_info_t peer={};
    memset(peer.peer_addr,0xFF,6); peer.channel=0; peer.encrypt=false;
    esp_now_add_peer(&peer);

    levyStep=levySample(); targetHeading=(float)random(0,360);
    lastLoop=millis();

    Serial.print("MAC: ");
    for(int i=0;i<6;i++){Serial.print(myMAC[i],HEX);if(i<5)Serial.print(":");}
    Serial.println();
    Serial.println("Ready");
}

void loop() {
    unsigned long now=millis();
    float dt=(now-lastLoop)/1000.0f;
    if(dt<=0||dt>0.5f) dt=0.01f;
    lastLoop=now;

    imu_update(dt);

    if(now-lastHB>HEARTBEAT_MS){ sendPkt(PKT_DATA); lastHB=now; }

    Serial.print("R:"); Serial.print(AngleRoll,1);
    Serial.print(" P:"); Serial.print(AnglePitch,1);
    Serial.print(" Stk:"); Serial.print(imu_stuck);
    Serial.print(" SOS:"); Serial.print(isSOS);
    Serial.print(" Hlp:"); Serial.print(isHelper);
    Serial.print(" Mrg:"); Serial.println(isMerged);

    if(isMerged)       { mergedRun(); return; }
    if(isSOS)          { sosBroadcast(); return; }
    if(isHelper)       { convergeToDest(); return; }
    levyWalk(dt);
}
