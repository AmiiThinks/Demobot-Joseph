void server(int port);

char *get_packet(int *length);

const char* response_to_special_client_request(char* request,int *response_len);
