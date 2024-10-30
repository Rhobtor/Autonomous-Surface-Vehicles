import mysql.connector
from mysql.connector import Error
import time


class MYSQL(object):
    def __init__(self, host,database,user,password,port):
        self.host=host
        self.database=database
        self.user=user
        self.password=password
        self.port=port

    def insert_data_to_db(self,query,data):
        try:
            # # Establish a connection to the database (modify with your MySQL settings)
            # connection = mysql.connector.connect(
            #     host='192.168.17.1',  # Replace with your database host
            #     database='wqp',  # Replace with your database name
            #     user='root',  # Replace with your MySQL username
            #     password='password'  # Replace with your MySQL password
            # )

            # Establish a connection to the database (modify with your MySQL settings)
            connection = mysql.connector.connect(
                host=self.host,  # Replace with your database host
                database=self.database,  # Replace with your database name
                user=self.user,  # Replace with your MySQL username
                password=self.password  # Replace with your MySQL password
            )

            cursor = connection.cursor()

            # Execute the insert query with the provided data
            cursor.execute(query, data)
            connection.commit()  # Commit the transaction

            print(f"Datas '{data}' inserted successfully.")
                
        except Error as e:
            print(f"Error while connecting to MySQL: {e}")
        finally:
            if connection.is_connected():
                cursor.close()
                connection.close()
                print("MySQL connection is closed.")
