import sqlite3

from models.user import User, UserInDB


def add_user(user: UserInDB):

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

    except sqlite3.IntegrityError as error:
        print('Unique key violation occurred -', error)
        raise sqlite3.IntegrityError

    except sqlite3.Error as error:
        print('Error occurred -', error)

    except Exception as e:
        print("Non SQL Exception -", e)

    finally:

        if sqliteConnection:
            sqliteConnection.close()
            print('SQLite Connection closed')

    return success_flag

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
    # TODO: Fix bug in error log: Table does not exist

    sqliteConnection = None

    try:
        sqliteConnection = sqlite3.connect("/var/lib/sqlite/users.db")
        cursor = sqliteConnection.cursor()
        print('DB: Init')

        # Check if the table exists
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='your_table_name';")
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
