<?php

$json = file_get_contents('php://input');
$data = json_decode($json);

$frameName = $data->frame_name;
$bid = $data->BID;
$wake_status = $data->WAKE_STATUS;
$tableName = "wake_status";
$column_list = "wake_status";
$value_list = $wake_status;


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