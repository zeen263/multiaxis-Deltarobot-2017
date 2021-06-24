# multiaxis-Deltarobot-2017

(소리가 좀 큽니다! 볼륨 주의)<br>


https://user-images.githubusercontent.com/42488309/123304269-b925cd00-d559-11eb-97af-dd3faa57a1f4.mp4





2017년에 기구 제어를 맡아 진행한 졸업작품으로, 기존 3축의 델타로봇 형태에 1축을 추가하여 4자유도 움직임 `x,y,z 이동과 roll 방향 회전(tilting)` 이 가능한 병렬 로봇입니다.


움직임 자유도를 추가함으로써 기존 3축의 델타로봇이 수행하는 pick & place 작업을 심화, 조립-이송-적재 작업을 효율적으로 수행할 수 있습니다. 작업물을 이동시켜 조립하고 다시 이동시켜 적재하는 과정을 통합하기 위해 작업 경로를 직선 이송과 tilting 회전이 결합된 형태로 설계했습니다. 로봇의 엔드 이펙터는 작업 경로를 따라 이동하는 표적을 추적하며 조립을 수행하고, 조립이 완료되면 작업물은 미끄러져 내려와 자동으로 적재되어 작업 시간 단축이 가능합니다.

다축 보간 제어를 지원하는 커미조아 사의 ceIP 타입 제어기를 사용했으며, 제공된 C# API를 이용해 프로그램을 구성했습니다.
