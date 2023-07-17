<?php

$json = file_get_contents('php://input');
$data = json_decode($json);

$frameName = $data->frame_name;
$bid = $data->BID;
$temperature = $data->TEMP;
$tableName = "temperature";
$column_list = "";
$value_list = "";

for ($x = 1; $x <= 9; $x++) {
  $column_list .= "t";
  $column_list .= $x;
  if ($x != 9) {
  	$column_list .= ",";
  } 
}

for ($x = 0; $x <= 8; $x++) {
  $value_list .= $temperature[$x];
  if ($x != 8) {
  	$value_list .= ",";
  } 
}


$columns = $column_list;
$values = $value_list;

// echo $column;
// echo $value;

$conn = mysqli_connect("localhost", "root", "", $frameName);
$sql = "INSERT INTO $tableName ($columns) VALUES ($values)";
echo $sql;
$result = mysqli_query($conn, $sql);

if (!$result) {
	echo mysql_error();
}

?>