#include <iostream>
#include <string>
#include <mysql/mysql.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

const int PORT = 8080;

void insertDataToDatabase(float temperature, float light, float sound, bool usage, float waterLevel) {
    MYSQL *conn;
    conn = mysql_init(NULL);

    // Connect to database
    if (!mysql_real_connect(conn, "localhost", "root", "", "environmentdata_db", 0, NULL, 0)) {
        std::cerr << "MySQL connection failed: " << mysql_error(conn) << std::endl;
        return;
    }

    // Prepare and execute SQL query
    std::string query = "INSERT INTO EnvironmentalData (temperature, ambient_light, sound_level, object_usage, water_level) VALUES (" +
                        std::to_string(temperature) + ", " +
                        std::to_string(light) + ", " +
                        std::to_string(sound) + ", " +
                        std::to_string(usage) + ", " +
                        std::to_string(waterLevel) + ")";
    if (mysql_query(conn, query.c_str())) {
        std::cerr << "INSERT failed: " << mysql_error(conn) << std::endl;
    }

    mysql_close(conn);
}

int main() {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // Create socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Attach socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt failed");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Bind the socket to the port
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // Listen for incoming connections
    if (listen(server_fd, 3) < 0) {
        perror("listen failed");
        exit(EXIT_FAILURE);
    }

    while (true) {
        std::cout << "Waiting for connection..." << std::endl;
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
            perror("accept failed");
            exit(EXIT_FAILURE);
        }

        char buffer[1024] = {0};
        int valread = read(new_socket, buffer, 1024);

        // Assume data format: temperature,light,sound,usage,waterLevel
        float temperature, light, sound, waterLevel;
        bool usage;
        sscanf(buffer, "%f,%f,%f,%d,%f", &temperature, &light, &sound, &usage, &waterLevel);

        insertDataToDatabase(temperature, light, sound, usage, waterLevel);
        close(new_socket);
    }
    return 0;
}
