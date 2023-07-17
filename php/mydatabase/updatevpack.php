<?php

$json = file_get_contents('php://input');
$data = json_decode($json);

$frameName = $data->frame_name;
$bid = $data->BID;
$vpack = $data->VPACK;
$tableName = "vpack";
$column_list = "";
$value_list = "";

for ($x = 1; $x <= 3; $x++) {
  $column_list .= "vp";
  $column_list .= $x;
  if ($x != 3) {
  	$column_list .= ",";
  } 
}

for ($x = 1; $x <= 3; $x++) {
  $value_list .= $vpack[$x];
  if ($x != 3) {
  	$value_list .= ",";
  } 
}


$columns = $column_list;
$values = $value_list;

$conn = mysqli_connect("localhost", "root", "", $frameName);
$sql = "INSERT INTO $tableName ($columns) VALUES ($values)";
echo $sql;
$result = mysqli_query($conn, $sql);

if (!$result) {
	echo mysql_error();
}
?>