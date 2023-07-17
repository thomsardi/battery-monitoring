<?php

$database_name = "data_storage";

function createDatabase() {
	global $database_name;
	$database_connection = mysqli_connect("localhost", "root", "");
	$sql = "CREATE DATABASE IF NOT EXISTS $database_name"; 
	$result = mysqli_query($database_connection, $sql);
	if(!$result) {
		return -1;
	}
	return 1;
}

function queryDatabase($sql_query) {
	global $database_name;
	$database_connection = mysqli_connect("localhost", "root", "", $database_name);
	if ($database_connection -> connect_errno) {
	  // echo "Failed to connect to MySQL: " . $conn -> connect_error;
	  // exit();
		return -1;
	}
	return $result = mysqli_query($database_connection, $sql_query);
}

function echo_ln($string) {
	echo $string;
	echo "\r\n";
}

?>