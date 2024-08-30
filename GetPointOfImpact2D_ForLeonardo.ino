#include <math.h>
#include <AbsMouse.h>

#define PIN_DEBUG_CHECK 6

/*
마이크 혹은 가속도센서 설치 위치(반드시 마이크를 탄착판에 밀착하여 설치)
센서 입력은 디지털 on/off 신호로 들어와야함
MIC1--------------------------MIC2
 |              +              |
 |                             |
 |-                           +|
 |                             |
 |              -              |
MIC3--------------------------MIC4
*/

//0,1,2,3,7 사용 가능
#define MIC_COUNT      4						//전체 마이크 갯수 (현재 알고리즘에서는 4개 고정)
#define PIN_MIC1       3            //외부 인터럽트 기능을 위해 고정
#define PIN_MIC2       2            //외부 인터럽트 기능을 위해 고정
#define PIN_MIC3       0            //외부 인터럽트 기능을 위해 고정
#define PIN_MIC4       1            //외부 인터럽트 기능을 위해 고정

//tuning
#define mic_fs (16000000ul/64)              //sampling rate [Hz]
int TimerPrescaleVal = (0 << CS12) | (1 << CS11) | (1 << CS10); //16000000/64
int mic_width   = 400;						          //마이크 사이 거리(너비) [mm]
int mic_height  = 300;						          //마이크 사이 거리(높이) [mm]
float v;                                  //탄착판에서 소리의 전달 속도  [mm/s]

int MonitorWidth = 1920;
int MonitorHeight = 1280;
uint32_t DelayForRemoveEcho = 200;		    //충격 인식 후 다시 인식 시작하기까지 딜레이 (잔향 제거)[millisecond]

//탄착 지점
float X = 0;
float Y = 0;

float MicInputResetTime;               //너무 오래된 입력을 사용하지 않기 위한 리셋타임, 1.2*대각선 길이/v로 계산[second]
//int MicInputResetTimeCheckCount;
uint32_t MicInputTime[MIC_COUNT];			    //마이크별 충격음을 인식했을 때 시간
char MicInputExist[MIC_COUNT];				    //마이크별 충경 인식 여부 (잔향 제거)
char MicInputCount = 0;                   //충격음을 인식한 마이크 개수
uint32_t NowTime = 0;                     //현재 코드에서는 반드시 4바이트여야 함
char MicInputOrder = 1;                   //마이크 들어온 순서를 쉽게 알기위한 변수

ISR(INT0_vect)
{
  if( MicInputExist[0] == 0 )
  {
    register unsigned int IntTime;
    IntTime = TCNT1;
    MicInputExist[0] = MicInputOrder++;
    MicInputTime[0] = NowTime + IntTime;
    MicInputCount++;
    EIMSK &= 0xFF - (1 << INT0);
  }
}

ISR(INT1_vect)
{
  if( MicInputExist[1] == 0 )
  {
    register unsigned int IntTime;
    IntTime = TCNT1;
    MicInputExist[1] = MicInputOrder++;
    MicInputTime[1] = NowTime + IntTime;
    MicInputCount++;
    EIMSK &= 0xFF - (1 << INT1);
  }
}

ISR(INT2_vect)
{
  if( MicInputExist[2] == 0 )
  {
    register unsigned int IntTime;
    IntTime = TCNT1;
    MicInputExist[2] = MicInputOrder++;
    MicInputTime[2] = NowTime + IntTime;
    MicInputCount++;
    EIMSK &= 0xFF - (1 << INT2);
  }
}

ISR(INT3_vect)
{
  if( MicInputExist[3] == 0 )
  {
    register unsigned int IntTime;
    IntTime = TCNT1;
    MicInputExist[3] = MicInputOrder++;
    MicInputTime[3] = NowTime + IntTime;
    MicInputCount++;
    EIMSK &= 0xFF - (1 << INT3);
  }
}

ISR(TIMER1_OVF_vect)
{
  NowTime += 0xFFFF;
	//NowTime변수의 오버플로우는 30분에 한번씩 발생하니 처리 안함
}

void InputOff()
{
  EIMSK = 0;  //인터럽트 금지
  TIMSK1 = 0;
  //TCCR1B &= 0xFF - ((0 << CS12) | (0 << CS11) | (0 << CS10));	//타이머 종료
}

void CalculateReset()
{
	int i;
  EIMSK = 0;  //인터럽트 금지
  TIMSK1 = 0;

	for(i = 0 ;i < MIC_COUNT;i++) MicInputExist[i] = 0;
	NowTime = 0;
	MicInputCount = 0;
  MicInputOrder = 1;
  //MicInputResetTimeCheckCount = 0;

  //External int 재허용
  EIFR |= (1 << INTF3) | (1 << INTF2) | (1 << INTF1) | (1 << INTF0);
  //EIMSK |= (1 << INT3) | (1 << INT2) | (1 << INT1) | (1 << INT0);

  //타이머 카운트 초기화
  TCCR1B &= 0xFF - ((1 << CS12) | (1 << CS11) | (1 << CS10));	//타이머 종료
  TCNT1 = 0;
  TIFR1 |= (1 << TOV1);
  TCCR1B |= TimerPrescaleVal;  //타이머 재시작

  //인터럽트 허용
  EIMSK = (1<<INT3) | (1<<INT2) | (1<<INT1) | (1<<INT0);
  TIMSK1 |= (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (1 << TOIE1);		//
}

void setup()
{
  int i;
  float T;

  cli();    //모든 인터럽트 금지
  for(i = 0 ;i < MIC_COUNT;i++) MicInputExist[i] = 0;
	NowTime = 0;
	MicInputCount = 0;
  
  T = 22;
  v = 1000 * 331.3 * sqrt( (T+273.15) / 273.15 );
  MicInputResetTime = (1.05 * sqrt(mic_width*mic_width + mic_height*mic_height)) / v;        //너무 오래된 입력을 사용하지 않기 위한 리셋타임[second]  //
  //MicInputResetTimeCheckCount = 0;

  pinMode(PIN_MIC1, INPUT);
  pinMode(PIN_MIC2, INPUT);
  pinMode(PIN_MIC3, INPUT);
  pinMode(PIN_MIC4, INPUT);
  //pinMode(PIN_DEBUG_CHECK, OUTPUT);

  Serial.begin(250000);

  //External int 초기화
  //INT0, INT1 , INT2, INT3 rising edge 검출
	EICRA = (1<<ISC31) | (1<<ISC30) | (1<<ISC21) | (1<<ISC20) | (1<<ISC11) | (1<<ISC10) | (1<<ISC01) | (1<<ISC00);
  EICRB = 0;
	EIMSK = (1<<INT3) | (1<<INT2) | (1<<INT1) | (1<<INT0);  //인터럽트 허용

  //타이머 초기화
  //normal모드, 오버플로우 인터럽트 허용
  //prescale 64 : 250kHz
  TCCR1A = (0 << WGM11) | (0 << WGM10);
  TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12);
  TCCR1C = 0;
  TCNT1 = 0;
  TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (1 << TOIE1);		//
  TCCR1B |= TimerPrescaleVal;

  sei();  //인터럽트 허용

  AbsMouse.init(MonitorWidth, MonitorHeight);
}

void CalculatePos(float t01, float t23, float t02, float width, float height, int *x, int *y)
{
  float X, Y, tmp;

  if( (abs(t01) < 1e-8) || (abs(t23) < 1e-8) )												//Y축 위에 존재
	{
		float S;
		X = 0.0;

		S = v*v*t02*t02;
		tmp = ((height*height+width*width)*S-S*S) / (4*height*height-4*S);
		if(tmp < 0)
  	{
			tmp = 0;
		}

		if(t02 > 0) Y = -sqrt( tmp );
		else Y = sqrt( tmp );
  }else if( (abs(t02) < 1e-8) || (abs(t01-t23) < 1e-8) )										//X축 위에 존재
	{
		float S;
		Y = 0;

		S = v*v*t23*t23;
		tmp = ((width*width+height*height)*S-S*S) / (4*width*width-4*S);
		if(tmp < 0)
		{
			tmp = 0;
		}

		if(t23 > 0) X = sqrt( tmp );
		else X = -sqrt( tmp );
	}else{
    float algo_A1,algo_B1,algo_A2,algo_B2, alpha, beta, gamma;
  
		algo_A1 = t01 * t01;
		algo_A2 = t23 * t23;
		algo_B1 = ( 4 * algo_A1 ) / ( width*width - v*v*algo_A1 );
		algo_B2 = ( 4 * algo_A2 ) / ( width*width - v*v*algo_A2 );
		alpha = algo_B1 - algo_B2;
		beta = -height * (algo_B1 + algo_B2);
		gamma = (algo_B1 - algo_B2) * height * height / 4 + algo_A1 - algo_A2;

		tmp = beta*beta-4*alpha*gamma;
		if(tmp < 0)
		{
			tmp = 0;
		}
		Y = (-beta - sqrt( tmp ) ) / (2 * alpha);

		tmp = 1 + ( 4 * (Y-height/2)*(Y-height/2) / ( width*width-v*v*t01*t01) );
		if(tmp < 0)
		{
			tmp = 0;
		}

		X = v*t01*sqrt( tmp ) / 2;
	}

  if(x)
  {
    *x = (int)(X+0.5);
	  if( *x > width/2)		*x = width/2;      //결과값 최대치 제한
	  else if( *x < -width/2)	*x = -width/2;
  }
		
  if(y)
  {
    *y = (int)(Y+0.5);
    if( *y > height/2)	*y = height/2;      //결과값 최대치 제한
		else if( *y < -height/2)	*y = -height/2;
  }
}

void loop()
{
  if(MicInputCount >= MIC_COUNT)
	{
    int i;
    float t01, t23, t02, t13;
    int32_t tmp2;
    int x, y;
    InputOff();
    //digitalWrite(PIN_DEBUG_CHECK, HIGH);

    #if 0
    //------------------마이크 시간차 계산
    tmp2 = MicInputTime[0] - MicInputTime[1];
		//t01 = tmp2 / mic_fs;
    Serial.println(tmp2);
    //-----------------------------------
		#else

    //------------------탄착 지점 계산 코드
    tmp2 = MicInputTime[0] - MicInputTime[1];
		t01 = ((float)tmp2) / mic_fs;
    tmp2 = MicInputTime[2] - MicInputTime[3];
		t23 = ((float)tmp2) / mic_fs;
    tmp2 = MicInputTime[0] - MicInputTime[2];
		t02 = ((float)tmp2) / mic_fs;

    CalculatePos(t01, t23, t02, (float)mic_width, (float)mic_height, &x, (int *)0);
    //-----------------------------------
    //3 -> 1
		//1 -> 2
		//4 -> 3
		//2 -> 4
    tmp2 = MicInputTime[2] - MicInputTime[0];
		t01 = ((float)tmp2) / mic_fs;
    tmp2 = MicInputTime[3] - MicInputTime[1];
		t23 = ((float)tmp2) / mic_fs;
    tmp2 = MicInputTime[2] - MicInputTime[3];
		t02 = ((float)tmp2) / mic_fs;

    CalculatePos(t01, t23, t02, (float)mic_height, (float)mic_width, &y, (int *)0);

    #if 0
    Serial.print("X:");
    Serial.println(x);
    Serial.print("Y:");
    Serial.println(y);
    #else
    AbsMouse.move(
      (((int32_t)(x + mic_width/2)) * MonitorWidth) / mic_width,
      (((int32_t)(mic_height/2 - y)) * MonitorHeight) / mic_height
    );
    AbsMouse.press(MOUSE_LEFT);
    AbsMouse.release(MOUSE_LEFT);
    //Serial.print("X:");
    //Serial.println( (((int32_t)(x + mic_width/2)) * MonitorWidth) / mic_width );
    //Serial.print("Y:");
    //Serial.println( (((int32_t)(-y + mic_height/2)) * MonitorHeight) / mic_height );
    #endif
    
    #endif

    delay(DelayForRemoveEcho);

		CalculateReset();																				//리셋
    //digitalWrite(PIN_DEBUG_CHECK, LOW);
	}
  else if(MicInputCount > 0)					//측정 시간이 오래되면 리셋
	{
    #if 1
    int i;
    for(i=0;i<MIC_COUNT;i++)
    {
      if(MicInputExist[i] == 1)
      {
        register unsigned int IntTime;
        uint32_t tmp2;
        IntTime = TCNT1;
        tmp2 = (NowTime + IntTime) - MicInputTime[i];
        if( tmp2 >= (MicInputResetTime*mic_fs) )
        {
          CalculateReset();
        }
        break;
      }
    }
    #endif
	}
}
