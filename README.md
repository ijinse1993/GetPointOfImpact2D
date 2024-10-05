기본 목적은 에어소프트건으로 BB탄을 쏴서 탄착지점을 PC에서 받기 위한 코드이지만 충격음만 난다면 다른 목적으로도 사용할 수 있습니다.<br>
제대로 된 정확도를 얻기 위해선 온도 설정, 판의 재질, 탄착음의 크기등 물리적인 조건이 매우 중요합니다.<br>

1. 부품 구매<br>
 필요한 최소한의 재료는 아래와 같습니다.<br>
  - 아두이노 레오나르도<br>
  - 입력신호의 크기에 따라 디지털 출력이 나오는 마이크 4개 ( 아두이노 사운드 센서모듈 [SZH-EK033] https://www.devicemart.co.kr/goods/view?no=1279095 )<br>
  - 마이크와 아두이노를 연결하고 전원을 공급할 전선<br>
  - 5V 전원 ( 아두이노 마이크로usb로 전원 공급 가능 )<br>
  - LM35D 온도 센서 혹은 온도계 (공기온도가 정확도에 상당히 큰 영향을 끼치므로 온도 센서를 설치하거나 온도를 다른 방법으로 측정하여 코드에 반영해주는 작업이 필요합니다) <br>
  - 이외에 모니터에 설치할 경우 투명연질PVC필름을 탄착판으로 사용할수있으며 필름 고정시키기 위한 프레임등을 모니터에 맞춰 추가 물품을 구비해주시면 됩니다. <br>
  이때 PVC필름은 모니터에서 약간 떨어져있어야하며 PVC필름은 스펀지를 사이에 두고 프레임에 연결하고 PVC필름을 약간 낭창낭창하게 하는등 최대한 PVC필름이 받는 충격이 프레임을 통해 마이크로 전달되지 않게 하는것이 중요합니다.<br>
  빔프로젝터를 이용해 벽을 탄착판으로 사용하는것 조건은 테스트되지 않았습니다. 과거 진행한 테스트 중 아크릴판으로 모니터에 고정했을때 정확도가 상당히 떨어져서 PVC필름으로 테스트를 진행하게 되었는데 이와 비슷한 문제가 벽에 설치 시 있을 수 있습니다.)<br>


2. PC 환경 구축<br>
PC에는 아두이노IDE가 설치되어야 하며 구글에 아두이노ide라고 검색하시면 받을 수 있는 페이지가 바로 나옵니다.
오른쪽에 Windows Win10 and newer, 64bits 를 클릭하시면 뭐라뭐라하는 페이지들이 나오는데 아래에 just download를 몇 번 눌러주시면 다운로드가 진행됩니다. 다운로드가 끝난 후 실행하여 설치해주시면 제가 공유한 아두이노 코드를 아두이노에 올릴 수 있는 IDE환경이 마련됩니다.<br><br>

기본 설치되어 있는 마우스 라이브러리는 상대위치를 입력하게 되어 있는것 같아 absmouse라는 라이브러리를 추가로 설치하여 절대좌표로 입력되게 하였습니다. absmouse라이브러리는 아두이노 라이브러리 매니저에서 설치가능합니다. Tools-Manage Libraries를 클릭하시면 왼쪽에 창이 생기는데 검색란에 absmouse라고 치시고 install 버튼을 클릭해주시면 됩니다.<br>
코드의 일부분을 본인 상황에 맞게 수정한 뒤 아두이노를 PC에 usb로 연결해주시고 상단의 Select Board에서 BOARDS는 Leonardo를 선택하시고 PORTS는 아두이노를 연결하면 생기는 포트를 선택해주시면 됩니다.<br>
그리고 좌측 상단에 체크 표시 바로 오른쪽에 있는 우측방향을 가리키는 화살표를 클릭해주시면 컴파일과 함께 작성한 프로그램이 아두이노에 다운로드되며 이후 아두이노를 usb에 연결하면 자동으로 기능이 실행됩니다.


3. 코드 수정<br>
제가 공유드린 코드를 더블클릭하시면 자동으로 아두이노 IDE로 실행되며 코드 상단에 아래 항목들만 수정해주시고 화살표를 눌러 다운로드를 해주시면 소프트웨어는 작업 끝입니다.<br>
다중 모니터 환경에서는 기본 모니터를 기준으로 동작됩니다.<br>
동작 전 필수로 변경되어야 하는 조건은 * 표시하였습니다.
mic_width : * 마이크를 사각형 모양으로 설치했을 때 가로 방향의 거리(mm단위)<br>
mic_height : * 마이크를 사각형 모양으로 설치했을 때 세로 방향의 거리(mm단위)<br>
MonitorWidth : * 모니터의 가로 해상도 (픽셀 단위)<br>
MonitorHeight : * 모니터의 세로 해상도 (픽셀 단위)<br>
Temperature : * 기온('C 단위). 기본 값으로 22도로 설정되어 있으며 사용할 환경에 맞춰 변경 필요. 아래 USING_LM35D_TEMP_SENSOR옵션으로 추가 설치된 센서를 통해 온도값 받을 수 있게 설정가능<br>
DelayForRemoveEcho : 충격음 인식 후 잔향이 사라질때까지 대기하는 시간 (millisecond 단위)<br>
WaitAllInputDelay : 4개의 마이크 중 일부만 충격음을 인식하고 나머지 마이크는 인식을 못했을 경우 계속 기다리는 상황에서 벗어나기 위한 딜레이 게인<br>
MouseClickTime : 마이크 클릭 후 뗄때까지 걸리는 시간 (millisecond 단위)<br>
MouseClickButton : 충격음 인식 시 클릭될 마우스 버튼 설정<br>
USING_LM35D_TEMP_SENSOR : LM35D 온도 센서 사용 여부 설정<br>
TempSensorPin : LM35D 센서 사용시 센서가 연결된 아두이노 핀위치<br>

4. 마이크 설치<br>
마이크는 탄착지점을 감싸는 직사각형 모양으로 설치가 되어야 하며 반드시 아래 순서로 아두이노에 연결되어야 합니다.<br>
MIC1 - 모니터 좌상단에 대응<br>
MIC2 - 모니터 우상단에 대응<br>
MIC3 - 모니터 좌하단에 대응<br>
MIC4 - 모니터 우하단에 대응<br>


MIC1--------------------------MIC2<br>
|　　　　　　　　　　　　　　　|<br>
|　　　　　　　　　　　　　　　|<br>
|　　　　　　탄착지점　　　　　|<br>
|　　　　　　　　　　　　　　　|<br>
|　　　　　　　　　　　　　　　|<br>
MIC3--------------------------MIC4<br>

MIC1의 신호선 DO 연결 위치 : 아두이노 3번핀<br>
MIC2의 신호선 DO 연결 위치 : 아두이노 2번핀<br>
MIC3의 신호선 DO 연결 위치 : 아두이노 0번핀<br>
MIC4의 신호선 DO 연결 위치 : 아두이노 1번핀<br>


아래 설명은 제가 사용한 마이크 기준이며 다른 마이크를 사용하실 분들은 상황에 맞게 작업해주시면 됩니다.<br>
마이크의 +단자에는 5V를 입력해주시면 되고 G단자에는 0V(그라운드)를 연결해주시면 됩니다. AO핀은 사용하지 않습니다.<br>

각 마이크는 탄착판에 가까이 붙어있어야 하며 평면으로부터 1cm이내의 거리에 있는것이 좋습니다.<br>
또한 탄착판이 BB탄에 맞을 때 진동이 공기중이 아닌 구조물을 통해 마이크로 들어가지 않아야 하므로 마이크를 직접 탄착판에 붙이는건 추천하지 않으며 저는 아래 테스트 할 때 프레임과 마이크 사이에 스펀지를 두고 테스트하는것이 중요한 방법이었습니다.<br>
마이크가 탄착판의 BB탄이 부딪치는 면에 설치되는게 유리할 수 있으나 2T의 PVC필름을 사용하였을 때 필름 뒤에 설치하여도 충분한 정확도가 나오는것을 확인하였습니다. 장전음등의 외부 소음으로 인한 오작동을 줄이기 위해 마이크를 탄착판 뒤에 설치하고 밀봉을 하는것이 좋은 방법일수 있습니다.<br><br>

마이크 수음부가 바라보는 방향별 성능 또한 평가되지 않았으며 제 평가 조건은 각 마이크를 탄착판 중앙을 바라보도록 설치하였습니다.<br><br>

마이크 설치가 끝난 뒤에 마이크에 전원을 입력하면 마이크에 불이 들어오는데 상시로 켜지는 led를 제외한 나머지 led가 조용한 상황에서는 켜지고 원하는 충격음에 꺼지도록 최대한 가변저항(파란색)을 정밀하게 조절해주시면 됩니다.<br>
다만 led가 꺼지는 순간이 짧아 잔상문제로 이게 꺼졌다 켜진건지 반응하지 않은건지 헷갈릴 수 있습니다. 이 경우 충격음을 발생시켰을 때 아두이노가 반응하는지 확인하여 꺼지는지를 확인할 수 있습니다. 모든 마이크가 적절하게 조절되어 충격음에만 반응하게 됐다면 충격음에 맞춰 아두이노보드에서 led가 반짝이는것을 볼 수 있습니다.<br>
주의하실 점으로 가변저항이 끝없이 돌아가므로 잘못된 방향이다 싶으면 반대방향으로 돌려야합니다. 다만 제대로 된 방향으로 돌리고 있어도 원하는 상태가 될때까지 상당히 많이 돌려야 해서 좀 피곤합니다..

