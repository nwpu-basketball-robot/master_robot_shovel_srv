<h2>控制篮球机器人铲子程序</h2></br>
从其它节点接收到控制信息后,转变为下位机协议，控制铲子<br/>

<h3>有以下控制方式</h3></br>
<ul>
	<li>id = 0 ------ 铲子从最低处到达最高点</li>
	<li>id = 1 ------ 铲子从最高点到最低点</li>
	<li>id = 2 ------ 铲子把球放入弹射装置，随后再达到最低点</li>
	<li>id = 3 ------ 铲子将球抬在空中（按比赛要求，已经取消控制方式）</li>
</ul>