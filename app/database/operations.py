import sqlite3
from typing import List

# from models.user import User, UserInDB
from ..core.schemas import User, UserInDB


def add_user(user: UserInDB):
    """
    Function to add a new user into the database

    param: UserInDB
    return: bool (success)
    exceptions: sqlite3 Integrity error / sqlite3 Error / other
    """
    success_flag = False
    sqliteConnection = None
    
    try:
        # Connect to DB and create a cursor
        sqliteConnection = sqlite3.connect("/var/lib/sqlite/users.db")
        cursor = sqliteConnection.cursor()
        print('DB Init')

        # Write a query to insert the user data into the table
        query = '''
        INSERT INTO users (username, hashed_password, disabled, blacklist, start_time, end_time)
        VALUES (?, ?, ?, ?, ?, ?)
        '''
        # Execute the query with the user data
        cursor.execute(query, (user.username, user.hashed_password, user.disabled, user.blacklist, user.start_time, user.end_time))
        sqliteConnection.commit()
        
        print('User added successfully to the database.')

        # Close the cursor
        cursor.close()

        success_flag = True

    # Catch  integrity error - unique key repetition
    except sqlite3.IntegrityError as error:
        print('Unique key violation occurred -', error)
        raise sqlite3.IntegrityError

    # Catch other SQL errors
    except sqlite3.Error as error:
        print('Error occurred -', error)

    # General exception
    except Exception as e:
        print("Non SQL Exception -", e)

    finally:

        if sqliteConnection:
            sqliteConnection.close()
            print('SQLite Connection closed')

    return success_flag

def get_users() -> List[User]:
    
        users: List[User] = []
        sqliteConnection = None
    
        try:
            sqliteConnection = sqlite3.connect("/var/lib/sqlite/users.db")
            cursor = sqliteConnection.cursor()
            print('Connected to DB')
    
            # Write a query to fetch all the users
            query = "SELECT * FROM users"
            cursor.execute(query)
            users_details = cursor.fetchall()
    
            # Check if users exist
            if users_details:
                # Create a User object with the fetched details
                for user_details in users_details:
                    user = User(username=user_details[0], hashed_password=user_details[1], disabled=user_details[2], blacklist=user_details[3], start_time=user_details[4], end_time=user_details[5])
                    users.append(user)
            else:
                print("DB: No users found")
            
        # Handle errors
        except sqlite3.Error as error:
            print('DB: Error occurred - ', error)
    
        
        except Exception as e:
            print("DB: Non SQL Exception -", e)
    
        finally:
    
            if sqliteConnection:
                sqliteConnection.close()
                print("DB: Connection Closed")
    
            return users


# TODO: Optimize this function to use get_user_in_db and remove the hashed_password
def get_user(username: str) -> User | None:

    user: User | None = None
    sqliteConnection = None

    try: 
        sqliteConnection = sqlite3.connect("/var/lib/sqlite/users.db")
        cursor = sqliteConnection.cursor()
        print('Connected to DB')

        # Write a query to fetch the user details based on the username
        query = "SELECT * FROM users WHERE username = ?"
        cursor.execute(query, (username,))
        user_details = cursor.fetchone()

        # Check if user exists
        if user_details:
            # Create a User object with the fetched details
            user = User(username=user_details[0], hashed_password=user_details[1], disabled=user_details[2], blacklist=user_details[3], start_time=user_details[4], end_time=user_details[5])
            print(user, type(user))
        else:
            print("DB: User", username, "not found")
        
    # Handle errors
    except sqlite3.Error as error:
        print('DB: Error occurred - ', error)

    
    except Exception as e:
        print("DB: Non SQL Exception -", e)

    finally:

        if sqliteConnection:
            sqliteConnection.close()
            print("DB: Connection Closed")

        return user


def get_user_in_db(username: str) -> UserInDB | None:

    user: User | None = None
    sqliteConnection = None

    try: 
        sqliteConnection = sqlite3.connect("/var/lib/sqlite/users.db")
        cursor = sqliteConnection.cursor()
        print('Connected to DB in get_user_in_db')

        # Write a query to fetch the user details based on the username
        query = "SELECT * FROM users WHERE username = ?"
        cursor.execute(query, (username,))
        user_details = cursor.fetchone()

        # Check if user exists
        if user_details:
            # Create a User object with the fetched details
            user = UserInDB(username=user_details[0], hashed_password=user_details[1], disabled=user_details[2], blacklist=user_details[3], start_time=user_details[4], end_time=user_details[5])
            print(user, type(user))
        else:
            print("DB: User", username, "not found")
        
    # Handle errors
    except sqlite3.Error as error:
        print('DB: Error occurred - ', error)

    
    except Exception as e:
        print("DB: Non SQL Exception -", e)

    finally:

        if sqliteConnection:
            sqliteConnection.close()
            print("DB: Connection Closed")

        return user



def init():
    sqliteConnection = None

    try:
        sqliteConnection = sqlite3.connect("/var/lib/sqlite/users.db")
        cursor = sqliteConnection.cursor()
        print('DB: Init')

        # Check if the table exists
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='users';")
        table_exists = cursor.fetchone()

        if table_exists:
            # Write a query and execute it with cursor
            query = 'select sqlite_version();'
            cursor.execute(query)
        else:
            print('Table does not exist.')
            # Create the table
            create_table_query = '''
            CREATE TABLE users (
                username TEXT PRIMARY KEY,
                hashed_password TEXT NOT NULL,
                disabled BOOL NOT NULL,
                blacklist BOOL NOT NULL,
                start_time TEXT,
                end_time TEXT
            );
            '''

            cursor.execute(create_table_query)
            sqliteConnection.commit()
            print('Table created successfully.')

            # Insert root user into the table
            root_user_query = '''
            INSERT INTO users
            VALUES ('root', '$2b$12$EixZaYVK1fsbw1ZfbX3OXePaWxn96p36WQoeG6Lruj3vjPGga31lW', 0, 0, 0, 0);
            '''
            cursor.execute(root_user_query)
            sqliteConnection.commit()
            print('DB: Root user created successfully.')

    # Handle errors
    except sqlite3.Error as error:
        print('DB: Error occurred - ', error)

    
    except Exception as e:
        print("DB: Non SQL Exception -", e)

    finally:

        if sqliteConnection:
            sqliteConnection.close()
            print('DB: SQLite Connection closed')


def allot_timeslot(username: str, start_time: str, end_time: str) -> User | None:
    
        sqliteConnection = None
        user = None
    
        try:
            sqliteConnection = sqlite3.connect("/var/lib/sqlite/users.db")
            cursor = sqliteConnection.cursor()
            print('DB: Init')
    
            # Update the start_time and end_time of the user
            query = "UPDATE users SET start_time = ?, end_time = ? WHERE username = ?"
            cursor.execute(query, (start_time, end_time, username))
            sqliteConnection.commit()
    
            # Check if any rows were affected
            if cursor.rowcount > 0:
                print("DB: User", username, "timeslot updated successfully")
            else:
                print("DB: User", username, "not found")
            
        # Handle errors
        except sqlite3.Error as error:
            print('DB: Error occurred - ', error)
            raise sqlite3.Error
        
        except Exception as e:
            print("DB: Non SQL Exception -", e)
            raise e
    
        finally:
    
            if sqliteConnection:
                sqliteConnection.close()
                print("DB: Connection Closed")
    
            return user