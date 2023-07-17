<?php

require 'function.php';

$json = file_get_contents('php://input');
$data = json_decode($json);

$vcell = $data->vcell;
$temp = $data->temp;
$pack = $data->pack;
$tableName = $data->table_name;

$columns = "";
$values = "";
$column_list = "";
$value_list = "";
$element_count = count($vcell);

for ($x = 0; $x < $element_count; $x++) {
  $column_list .= "v";
  $column_list .= $x+1;
  $value_list .= $vcell[$x];
  if ($x != ($element_count - 1)) 
  {
    $column_list .= ",";
    $value_list .= ",";
  } 
}

$columns .= $column_list;
$columns .= ",";

$values .= $value_list;
$values .= ",";

$column_list = "";
$value_list = "";
$element_count = count($temp);
for ($x = 0; $x < $element_count; $x++) {
  $column_list .= "t";
  $column_list .= $x+1;
  $value_list .= $temp[$x];
  if ($x != ($element_count - 1)) {
    $column_list .= ",";
    $value_list .= ",";
  } 
}

$columns .= $column_list;
$columns .= ",";

$values .= $value_list;
$values .= ",";

$column_list = "";
$value_list = "";
$element_count = count($pack);
for ($x = 0; $x < $element_count; $x++) {
  $column_list .= "vp";
  $column_list .= $x+1;
  $value_list .= $pack[$x];
  if ($x != ($element_count-1)) {
    $column_list .= ",";
    $value_list .= ",";
  } 
}

$columns .= $column_list;
$values .= $value_list;

$column_list = "";
$value_list = "";


// $conn = mysqli_connect("localhost", "root", "", $databaseName);
$sql = "INSERT INTO `$tableName` ($columns) VALUES ($values)";
echo $sql;
// $result = mysqli_query($conn, $sql);
$result = queryDatabase($sql);

if (!$result) {
	echo mysql_error();
}

?>