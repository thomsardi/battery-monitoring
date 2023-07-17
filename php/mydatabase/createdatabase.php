<?php 

require 'function.php';

// $conn = mysqli_connect("localhost", "root", "");
$json = file_get_contents('php://input');
$data = json_decode($json);
// $vcell = $data->cms_data[0]->vcell;
$databaseName = "data_storage";
$tableName = $data->table_name;

// if (!is_string($tableName)) {
//   $temp = strval($tableName);
//   $tableName = "'" . $temp . "'";
// }

/*
$sql = "CREATE DATABASE IF NOT EXISTS $databaseName";

$result = mysqli_query($conn, $sql);

if (!$result) {
	echo mysql_error();
}

$conn = mysqli_connect("localhost", "root", "", $databaseName);

if ($conn -> connect_errno) {
  echo "Failed to connect to MySQL: " . $conn -> connect_error;
  exit();
}*/

$result = createDatabase();

if($result < 0)
{
  echo_ln("Could not connect to $databaseName database");
  echo_ln("Exit");
  exit();
}
else {
  echo_ln("$databaseName database created successfully");
}

$sql_create_table = "CREATE TABLE `$tableName` (
 `id` int(11) NOT NULL AUTO_INCREMENT,
 `timestamp` timestamp NOT NULL DEFAULT current_timestamp() ON UPDATE current_timestamp(),
 `v1` int(11) NOT NULL,
 `v2` int(11) NOT NULL,
 `v3` int(11) NOT NULL,
 `v4` int(11) NOT NULL,
 `v5` int(11) NOT NULL,
 `v6` int(11) NOT NULL,
 `v7` int(11) NOT NULL,
 `v8` int(11) NOT NULL,
 `v9` int(11) NOT NULL,
 `v10` int(11) NOT NULL,
 `v11` int(11) NOT NULL,
 `v12` int(11) NOT NULL,
 `v13` int(11) NOT NULL,
 `v14` int(11) NOT NULL,
 `v15` int(11) NOT NULL,
 `t1` int(11) NOT NULL,
 `t2` int(11) NOT NULL,
 `t3` int(11) NOT NULL,
 `t4` int(11) NOT NULL,
 `t5` int(11) NOT NULL,
 `t6` int(11) NOT NULL,
 `t7` int(11) NOT NULL,
 `t8` int(11) NOT NULL,
 `t9` int(11) NOT NULL,
 `vp1` int(11) NOT NULL,
 PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_general_ci";

$query = "SHOW TABLES LIKE '$tableName'";
$result = queryDatabase($query);
$tableExist = $result->num_rows;
// echo $query;
// $result = mysqli_query($conn, $query);
// $tableExist = $result->num_rows;

if($tableExist <= 0) {
  // $result = mysqli_query($conn, $sql_create_table);
  // echo "Table Not Exist";
  $result = queryDatabase($sql_create_table);
  if ($result) {
    echo_ln("$tableName table created successfully");
  }
}
else {
  echo_ln("$tableName Table Exist");
}

?>