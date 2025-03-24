1. Vertical dragging

첫 번째로 구현한 것은 vertical dragging이다.
onMouseDrag 함수에서 isDrag == V_DRAG일 때 vertical dragging을 수행할 수 있도록 하였다.
ray와 plane의 교점을 구하기 위해 먼저 horizontal dragging에서 currpos를 glob_pos 전역변수를 이용해 받아왔다.
ray의 parametric equation이 p + td 이기 때문에 t = (glob_pos - p)/d 가 된다.
구한 t를 이용하여 plane의 위치를 구함으로써 vertical dragging을 하게 되었다.

2. Horizontal dragging (fixed)
vertical dragging 후, 그 plane은 유지하면서 cow가 horizontal하게 움직여야 한다. 이를 위해 global 변수 (glob_plane_pos)를 추가하여 plane의 position을 저장해주었다.
Horizontal dragging에서는 plane을 생성할 때 주어진 glob_plane_pos를 이용해 plane을 생성하도록 하였다.

3. glob_cow_pos & cows
6번의 마우스 클릭을 이용해 cow의 Position을 저장하고 이를 보여주어야 한다. 
먼저 onMouseButton 함수에서 state가 GLFW_UP일 때 6번의 클릭에 대한 처리를 해 주었다.
전역변수로 cnt를 추가하여 count가 6이 될 때까지 glob_cow_pos(전역변수)에 cow의 position을 append해주었다.
cows에는 실질적인 cow의 위치를 저장해 주었고 glob_cow_pos에는 drawCow를 위한 cow의 위치가 저장되었다.
만약 cnt가 6에 도달하면 전역변수 start에 glfw.get_time()함수를 통해 시간을 받아오고, 전역변수 isrotating을 True로 만들어준다.

4. display
display에서 6번의 클릭동안 cow의 모습이 띄워지도록 조정해주었다.
isrotating이 아닌 경우 glob_cow_pos에서 원소를 꺼내 drawCow 함수를 이용해 cow를 그려줬다.
만약 cnt == 6이 되어 isrotating이 True가 되면 다음 과정을 수행한다.
a. Catmull-Rom spline curve를 구현하기 위해 먼저 spline coefficient를 구하기 위한 matrix를 두 개 만든다.
b. 그리고 현재 시간(currtime)을 시작시간에서 빼주어 상대적인 시간을 사용한다.
c. 시간에 따른 spline coefficient를 구해준다.
d. Catmull-Rome spline curve는 cubic이기에 matrix multiplication으로 해당 시간에 맞는 cow의 position을 구해준다.
e. 또한 미분을 이용하여 각각의 시간에서 spline curve에서의 방향을 구한다.
f. 이를 이용해 rotate를 수행한다.
g. 위 과정을 3번(18의 사간)동안 반복하고 변수들을 초기화하며 다시금 초기 상태로 돌아온다.

