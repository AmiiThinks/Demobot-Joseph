

// thread safe fast logging
void start_log(const char *filename, const char *header,int max_line_size, int ms_sleep, int num_lines);
void writelog(const char *line);
void stoplog();
