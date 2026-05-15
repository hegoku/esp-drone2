#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#ifndef _DARWIN_C_SOURCE
#define _DARWIN_C_SOURCE
#endif

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>
#endif

#define ANOTC_V8_HEAD 0xAB
#define ANOTC_V8_HEAD_SIZE 6
#define ANOTC_DATA_MAX_SIZE 256
#define ANOTC_DEVICE_ADDR 0x01
#define ANOTC_PC_ADDR 0xFE
#define ANOTC_FRAME_LOG_STRING 0xA0
#define ANOTC_FRAME_CONFIG_CMD 0xE0
#define ANOTC_CONFIG_FRAME_CMD_DEVICE_INFO 0x00
#define CSV_HEADER "timestamp,\"frame_id\",\"message\",\"payload_hex\"\n"
#define MAX_FRAME_PARAMS 64
#define MAX_DATA_COLUMNS 256
#define MAX_NAME_SIZE 64

enum field_type {
	FIELD_INT8 = 0,
	FIELD_UINT8,
	FIELD_INT16,
	FIELD_UINT16,
	FIELD_INT32,
	FIELD_UINT32,
	FIELD_INT64,
	FIELD_UINT64,
	FIELD_FLOAT,
	FIELD_DOUBLE
};

struct frame_param_def {
	char name[MAX_NAME_SIZE];
	enum field_type type;
	size_t size;
	double scale;
	int column_index;
};

struct frame_def {
	int present;
	uint8_t id;
	char name[MAX_NAME_SIZE];
	int param_count;
	struct frame_param_def params[MAX_FRAME_PARAMS];
};

struct frame_definitions {
	int enabled;
	int column_count;
	char columns[MAX_DATA_COLUMNS][MAX_NAME_SIZE];
	struct frame_def frames[256];
};

enum parser_state {
	STATE_HEAD = 0,
	STATE_SRC_ADDR,
	STATE_DST_ADDR,
	STATE_FUNC,
	STATE_LEN1,
	STATE_LEN2,
	STATE_DATA,
	STATE_SUM_CHECK,
	STATE_ADD_CHECK
};

struct options {
	const char *device_path;
	const char *output_path;
	const char *frame_def_path;
	int baud_rate;
	int append;
	int quiet;
	int verbose;
	int no_connect;
	int receive_all_frames;
	int frame_filter_count;
	uint8_t frame_filter[256];
};

struct writer {
	FILE *file;
	int echo_stdout;
};

struct parser {
	enum parser_state state;
	uint8_t frame[ANOTC_V8_HEAD_SIZE + ANOTC_DATA_MAX_SIZE];
	size_t stored_len;
	uint16_t expected_len;
	uint16_t data_read;
	uint8_t expected_sum;
	uint8_t expected_add;
	uint8_t computed_sum;
	uint8_t computed_add;
	uint64_t bytes_received;
	uint64_t total_frames;
	uint64_t matched_frames;
	uint64_t seen_frames[256];
	uint64_t checksum_errors;
	uint64_t oversize_frames;
};

static volatile sig_atomic_t g_should_stop = 0;

static void debug_print_hex(const char *prefix, const uint8_t *data, size_t len);

static void handle_signal(int signo)
{
	(void)signo;
	g_should_stop = 1;
}

static int install_signal_handlers(void)
{
	struct sigaction action;

	memset(&action, 0, sizeof(action));
	action.sa_handler = handle_signal;
	sigemptyset(&action.sa_mask);

	if (sigaction(SIGINT, &action, NULL) != 0) {
		perror("sigaction(SIGINT)");
		return -1;
	}
	if (sigaction(SIGTERM, &action, NULL) != 0) {
		perror("sigaction(SIGTERM)");
		return -1;
	}

	return 0;
}

static void print_usage(FILE *stream, const char *program)
{
	fprintf(
		stream,
		"Usage: %s -d <serial-device> [options]\n"
		"\n"
		"Receive ANOTC frames from a serial port and write them to a CSV log file.\n"
		"\n"
		"Options:\n"
		"  -d, --device <path>      Serial device path, for example /dev/ttyUSB0\n"
		"  -b, --baud <rate>        Baud rate, default 460800\n"
		"  -f, --frame-id <id>      Frame function ID to log, default 0xA0; repeatable, or use all\n"
		"  -j, --frame-def <path>   JSON frame definition file for typed CSV output\n"
		"  -o, --output <path>      CSV log file path, default ./anotc.csv\n"
		"  -a, --append             Append to the log file instead of truncating it\n"
		"  -q, --quiet              Do not mirror decoded logs to stdout\n"
		"  -t, --timestamp          Accepted for compatibility; CSV timestamps are always written\n"
		"  -v, --verbose            Print serial/debug information to stderr\n"
		"  -n, --no-connect         Do not send the ANOTC device-info connect request\n"
		"  -h, --help               Show this help message\n",
		program
	);
}

static int parse_int(const char *text, int *value)
{
	char *end = NULL;
	long parsed = 0;

	if (text == NULL || *text == '\0') {
		return -1;
	}

	errno = 0;
	parsed = strtol(text, &end, 10);
	if (errno != 0 || end == text || *end != '\0' || parsed <= 0 || parsed > INT32_MAX) {
		return -1;
	}

	*value = (int)parsed;
	return 0;
}

static int parse_frame_id(const char *text, uint8_t *frame_id)
{
	char *end = NULL;
	long parsed = 0;
	int base = 10;

	if (text == NULL || *text == '\0') {
		return -1;
	}

	if (text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
		base = 16;
	} else {
		for (const char *p = text; *p != '\0'; p++) {
			if ((*p >= 'a' && *p <= 'f') || (*p >= 'A' && *p <= 'F')) {
				base = 16;
				break;
			}
		}
	}

	errno = 0;
	parsed = strtol(text, &end, base);
	if (errno != 0 || end == text || *end != '\0' || parsed < 0 || parsed > UINT8_MAX) {
		return -1;
	}

	*frame_id = (uint8_t)parsed;
	return 0;
}

static int add_frame_filter(struct options *opts, const char *text)
{
	uint8_t frame_id = 0;

	if (strcmp(text, "all") == 0 || strcmp(text, "ALL") == 0) {
		opts->receive_all_frames = 1;
		return 0;
	}

	if (parse_frame_id(text, &frame_id) != 0) {
		fprintf(stderr, "Invalid frame ID: %s\n", text);
		return -1;
	}

	if (!opts->frame_filter[frame_id]) {
		opts->frame_filter[frame_id] = 1;
		opts->frame_filter_count++;
	}

	return 0;
}

static int parse_options(int argc, char **argv, struct options *opts)
{
	int ch = 0;
	static const struct option long_options[] = {
		{"device", required_argument, NULL, 'd'},
		{"baud", required_argument, NULL, 'b'},
		{"frame-id", required_argument, NULL, 'f'},
		{"frame-def", required_argument, NULL, 'j'},
		{"output", required_argument, NULL, 'o'},
		{"append", no_argument, NULL, 'a'},
		{"quiet", no_argument, NULL, 'q'},
		{"timestamp", no_argument, NULL, 't'},
		{"verbose", no_argument, NULL, 'v'},
		{"no-connect", no_argument, NULL, 'n'},
		{"help", no_argument, NULL, 'h'},
		{0, 0, 0, 0}
	};

	memset(opts, 0, sizeof(*opts));
	opts->baud_rate = 460800;
	opts->output_path = "anotc.csv";

	while ((ch = getopt_long(argc, argv, "d:b:f:j:o:aqtvnh", long_options, NULL)) != -1) {
		switch (ch) {
		case 'd':
			opts->device_path = optarg;
			break;
		case 'b':
			if (parse_int(optarg, &opts->baud_rate) != 0) {
				fprintf(stderr, "Invalid baud rate: %s\n", optarg);
				return -1;
			}
			break;
		case 'f':
			if (add_frame_filter(opts, optarg) != 0) {
				return -1;
			}
			break;
		case 'j':
			opts->frame_def_path = optarg;
			break;
		case 'o':
			opts->output_path = optarg;
			break;
		case 'a':
			opts->append = 1;
			break;
		case 'q':
			opts->quiet = 1;
			break;
		case 't':
			break;
		case 'v':
			opts->verbose = 1;
			break;
		case 'n':
			opts->no_connect = 1;
			break;
		case 'h':
			print_usage(stdout, argv[0]);
			return 1;
		default:
			print_usage(stderr, argv[0]);
			return -1;
		}
	}

	if (opts->device_path == NULL) {
		fprintf(stderr, "Serial device is required.\n");
		print_usage(stderr, argv[0]);
		return -1;
	}

	if (strcmp(opts->output_path, "-") == 0 && opts->quiet) {
		fprintf(stderr, "Refusing to discard all output: use a file path or remove --quiet.\n");
		return -1;
	}

	return 0;
}

static int string_equal_ignore_case(const char *a, const char *b)
{
	while (*a != '\0' && *b != '\0') {
		char ca = *a;
		char cb = *b;

		if (ca >= 'A' && ca <= 'Z') {
			ca = (char)(ca - 'A' + 'a');
		}
		if (cb >= 'A' && cb <= 'Z') {
			cb = (char)(cb - 'A' + 'a');
		}
		if (ca != cb) {
			return 0;
		}
		a++;
		b++;
	}

	return *a == '\0' && *b == '\0';
}

static int field_type_from_name(const char *name, enum field_type *type, size_t *size)
{
	if (string_equal_ignore_case(name, "Int8")) {
		*type = FIELD_INT8;
		*size = 1;
		return 0;
	}
	if (string_equal_ignore_case(name, "UInt8") || string_equal_ignore_case(name, "Uint8")) {
		*type = FIELD_UINT8;
		*size = 1;
		return 0;
	}
	if (string_equal_ignore_case(name, "Int16")) {
		*type = FIELD_INT16;
		*size = 2;
		return 0;
	}
	if (string_equal_ignore_case(name, "UInt16") || string_equal_ignore_case(name, "Uint16")) {
		*type = FIELD_UINT16;
		*size = 2;
		return 0;
	}
	if (string_equal_ignore_case(name, "Int32")) {
		*type = FIELD_INT32;
		*size = 4;
		return 0;
	}
	if (string_equal_ignore_case(name, "UInt32") || string_equal_ignore_case(name, "Uint32")) {
		*type = FIELD_UINT32;
		*size = 4;
		return 0;
	}
	if (string_equal_ignore_case(name, "Int64")) {
		*type = FIELD_INT64;
		*size = 8;
		return 0;
	}
	if (string_equal_ignore_case(name, "UInt64") || string_equal_ignore_case(name, "Uint64")) {
		*type = FIELD_UINT64;
		*size = 8;
		return 0;
	}
	if (string_equal_ignore_case(name, "Float") || string_equal_ignore_case(name, "Float32")) {
		*type = FIELD_FLOAT;
		*size = 4;
		return 0;
	}
	if (string_equal_ignore_case(name, "Double") || string_equal_ignore_case(name, "Float64")) {
		*type = FIELD_DOUBLE;
		*size = 8;
		return 0;
	}

	return -1;
}

static void json_skip_ws(const char **cursor)
{
	while (**cursor == ' ' || **cursor == '\n' || **cursor == '\r' || **cursor == '\t') {
		(*cursor)++;
	}
}

static int json_expect_char(const char **cursor, char expected)
{
	json_skip_ws(cursor);
	if (**cursor != expected) {
		return -1;
	}
	(*cursor)++;
	return 0;
}

static int json_parse_string(const char **cursor, char *out, size_t out_size)
{
	size_t out_len = 0;

	json_skip_ws(cursor);
	if (**cursor != '"') {
		return -1;
	}
	(*cursor)++;

	while (**cursor != '\0' && **cursor != '"') {
		char ch = **cursor;

		if (ch == '\\') {
			(*cursor)++;
			ch = **cursor;
			if (ch == '\0') {
				return -1;
			}
			switch (ch) {
			case '"':
			case '\\':
			case '/':
				break;
			case 'b':
				ch = '\b';
				break;
			case 'f':
				ch = '\f';
				break;
			case 'n':
				ch = '\n';
				break;
			case 'r':
				ch = '\r';
				break;
			case 't':
				ch = '\t';
				break;
			case 'u':
				for (int i = 0; i < 4; i++) {
					(*cursor)++;
					if (**cursor == '\0') {
						return -1;
					}
				}
				ch = '?';
				break;
			default:
				return -1;
			}
		}

		if (out_len + 1 < out_size) {
			out[out_len++] = ch;
		}
		(*cursor)++;
	}

	if (**cursor != '"') {
		return -1;
	}
	(*cursor)++;
	if (out_size > 0) {
		out[out_len] = '\0';
	}

	return 0;
}

static int json_parse_int(const char **cursor, int *value)
{
	char *end = NULL;
	long parsed = 0;

	json_skip_ws(cursor);
	errno = 0;
	parsed = strtol(*cursor, &end, 10);
	if (errno != 0 || end == *cursor || parsed < INT32_MIN || parsed > INT32_MAX) {
		return -1;
	}

	*value = (int)parsed;
	*cursor = end;
	return 0;
}

static int json_parse_double(const char **cursor, double *value)
{
	char *end = NULL;
	double parsed = 0.0;

	json_skip_ws(cursor);
	errno = 0;
	parsed = strtod(*cursor, &end);
	if (errno != 0 || end == *cursor) {
		return -1;
	}

	*value = parsed;
	*cursor = end;
	return 0;
}

static int json_skip_value(const char **cursor);

static int json_skip_array(const char **cursor)
{
	if (json_expect_char(cursor, '[') != 0) {
		return -1;
	}
	json_skip_ws(cursor);
	if (**cursor == ']') {
		(*cursor)++;
		return 0;
	}

	for (;;) {
		if (json_skip_value(cursor) != 0) {
			return -1;
		}
		json_skip_ws(cursor);
		if (**cursor == ']') {
			(*cursor)++;
			return 0;
		}
		if (**cursor != ',') {
			return -1;
		}
		(*cursor)++;
	}
}

static int json_skip_object(const char **cursor)
{
	char key[MAX_NAME_SIZE];

	if (json_expect_char(cursor, '{') != 0) {
		return -1;
	}
	json_skip_ws(cursor);
	if (**cursor == '}') {
		(*cursor)++;
		return 0;
	}

	for (;;) {
		if (json_parse_string(cursor, key, sizeof(key)) != 0 ||
		    json_expect_char(cursor, ':') != 0 ||
		    json_skip_value(cursor) != 0) {
			return -1;
		}
		json_skip_ws(cursor);
		if (**cursor == '}') {
			(*cursor)++;
			return 0;
		}
		if (**cursor != ',') {
			return -1;
		}
		(*cursor)++;
	}
}

static int json_skip_value(const char **cursor)
{
	char scratch[MAX_NAME_SIZE];
	char *end = NULL;

	json_skip_ws(cursor);
	if (**cursor == '"') {
		return json_parse_string(cursor, scratch, sizeof(scratch));
	}
	if (**cursor == '{') {
		return json_skip_object(cursor);
	}
	if (**cursor == '[') {
		return json_skip_array(cursor);
	}
	if (strncmp(*cursor, "true", 4) == 0) {
		*cursor += 4;
		return 0;
	}
	if (strncmp(*cursor, "false", 5) == 0) {
		*cursor += 5;
		return 0;
	}
	if (strncmp(*cursor, "null", 4) == 0) {
		*cursor += 4;
		return 0;
	}

	errno = 0;
	(void)strtod(*cursor, &end);
	if (errno != 0 || end == *cursor) {
		return -1;
	}
	*cursor = end;
	return 0;
}

static int parse_param_object(const char **cursor, struct frame_param_def *param)
{
	char key[MAX_NAME_SIZE];
	char type_name[MAX_NAME_SIZE];
	int has_name = 0;
	int has_type = 0;

	memset(param, 0, sizeof(*param));
	param->scale = 1.0;
	param->column_index = -1;

	if (json_expect_char(cursor, '{') != 0) {
		return -1;
	}
	json_skip_ws(cursor);
	if (**cursor == '}') {
		(*cursor)++;
		return -1;
	}

	for (;;) {
		if (json_parse_string(cursor, key, sizeof(key)) != 0 ||
		    json_expect_char(cursor, ':') != 0) {
			return -1;
		}

		if (strcmp(key, "name") == 0) {
			if (json_parse_string(cursor, param->name, sizeof(param->name)) != 0) {
				return -1;
			}
			has_name = 1;
		} else if (strcmp(key, "type") == 0) {
			if (json_parse_string(cursor, type_name, sizeof(type_name)) != 0 ||
			    field_type_from_name(type_name, &param->type, &param->size) != 0) {
				fprintf(stderr, "Unsupported frame definition type: %s\n", type_name);
				return -1;
			}
			has_type = 1;
		} else if (strcmp(key, "scale") == 0) {
			if (json_parse_double(cursor, &param->scale) != 0) {
				return -1;
			}
		} else if (strcmp(key, "divider") == 0 || strcmp(key, "divisor") == 0) {
			double divider = 0.0;

			if (json_parse_double(cursor, &divider) != 0 || divider == 0.0) {
				return -1;
			}
			param->scale = 1.0 / divider;
		} else {
			if (json_skip_value(cursor) != 0) {
				return -1;
			}
		}

		json_skip_ws(cursor);
		if (**cursor == '}') {
			(*cursor)++;
			break;
		}
		if (**cursor != ',') {
			return -1;
		}
		(*cursor)++;
	}

	return (has_name && has_type) ? 0 : -1;
}

static int parse_params_array(const char **cursor, struct frame_def *frame)
{
	if (json_expect_char(cursor, '[') != 0) {
		return -1;
	}
	json_skip_ws(cursor);
	if (**cursor == ']') {
		(*cursor)++;
		return 0;
	}

	for (;;) {
		struct frame_param_def param;

		if (frame->param_count >= MAX_FRAME_PARAMS) {
			fprintf(stderr, "Too many params in frame id %u.\n", frame->id);
			return -1;
		}
		if (parse_param_object(cursor, &param) != 0) {
			return -1;
		}
		frame->params[frame->param_count++] = param;

		json_skip_ws(cursor);
		if (**cursor == ']') {
			(*cursor)++;
			return 0;
		}
		if (**cursor != ',') {
			return -1;
		}
		(*cursor)++;
	}
}

static int parse_frame_object(const char **cursor, struct frame_definitions *defs)
{
	struct frame_def frame;
	char key[MAX_NAME_SIZE];
	int id = -1;

	memset(&frame, 0, sizeof(frame));

	if (json_expect_char(cursor, '{') != 0) {
		return -1;
	}
	json_skip_ws(cursor);
	if (**cursor == '}') {
		(*cursor)++;
		return 0;
	}

	for (;;) {
		if (json_parse_string(cursor, key, sizeof(key)) != 0 ||
		    json_expect_char(cursor, ':') != 0) {
			return -1;
		}

		if (strcmp(key, "id") == 0) {
			if (json_parse_int(cursor, &id) != 0 || id < 0 || id > UINT8_MAX) {
				fprintf(stderr, "Frame definition id must be in 0..255.\n");
				return -1;
			}
			frame.id = (uint8_t)id;
		} else if (strcmp(key, "name") == 0) {
			if (json_parse_string(cursor, frame.name, sizeof(frame.name)) != 0) {
				return -1;
			}
		} else if (strcmp(key, "params") == 0) {
			if (parse_params_array(cursor, &frame) != 0) {
				return -1;
			}
		} else {
			if (json_skip_value(cursor) != 0) {
				return -1;
			}
		}

		json_skip_ws(cursor);
		if (**cursor == '}') {
			(*cursor)++;
			break;
		}
		if (**cursor != ',') {
			return -1;
		}
		(*cursor)++;
	}

	if (id >= 0 && frame.param_count > 0) {
		frame.present = 1;
		defs->frames[frame.id] = frame;
	}

	return 0;
}

static int read_text_file(const char *path, char **content)
{
	FILE *file = fopen(path, "rb");
	long size = 0;
	char *buffer = NULL;

	if (file == NULL) {
		perror(path);
		return -1;
	}
	if (fseek(file, 0, SEEK_END) != 0) {
		perror("fseek");
		fclose(file);
		return -1;
	}
	size = ftell(file);
	if (size < 0) {
		perror("ftell");
		fclose(file);
		return -1;
	}
	if (fseek(file, 0, SEEK_SET) != 0) {
		perror("fseek");
		fclose(file);
		return -1;
	}

	buffer = malloc((size_t)size + 1);
	if (buffer == NULL) {
		perror("malloc");
		fclose(file);
		return -1;
	}

	if (fread(buffer, 1, (size_t)size, file) != (size_t)size) {
		perror("fread");
		free(buffer);
		fclose(file);
		return -1;
	}
	buffer[size] = '\0';
	fclose(file);

	*content = buffer;
	return 0;
}

static int load_frame_definitions(const char *path, struct frame_definitions *defs)
{
	char *content = NULL;
	const char *cursor = NULL;

	memset(defs, 0, sizeof(*defs));
	if (path == NULL) {
		return 0;
	}

	if (read_text_file(path, &content) != 0) {
		return -1;
	}
	cursor = content;

	if (json_expect_char(&cursor, '[') != 0) {
		fprintf(stderr, "Frame definition JSON root must be an array.\n");
		free(content);
		return -1;
	}
	json_skip_ws(&cursor);
	if (*cursor != ']') {
		for (;;) {
			if (parse_frame_object(&cursor, defs) != 0) {
				fprintf(stderr, "Failed to parse frame definition JSON near: %.32s\n", cursor);
				free(content);
				return -1;
			}
			json_skip_ws(&cursor);
			if (*cursor == ']') {
				break;
			}
			if (*cursor != ',') {
				fprintf(stderr, "Expected ',' or ']' in frame definition JSON.\n");
				free(content);
				return -1;
			}
			cursor++;
		}
	}
	cursor++;
	json_skip_ws(&cursor);
	if (*cursor != '\0') {
		fprintf(stderr, "Unexpected trailing data in frame definition JSON.\n");
		free(content);
		return -1;
	}

	defs->enabled = 1;
	free(content);
	return 0;
}

static int ensure_frame_filters(struct options *opts, const struct frame_definitions *defs)
{
	if (opts->receive_all_frames || opts->frame_filter_count > 0) {
		return 0;
	}

	if (defs->enabled) {
		for (int id = 0; id < 256; id++) {
			if (defs->frames[id].present) {
				opts->frame_filter[id] = 1;
				opts->frame_filter_count++;
			}
		}
		if (opts->frame_filter_count == 0) {
			fprintf(stderr, "Frame definition file did not contain any usable frames.\n");
			return -1;
		}
		return 0;
	}

	opts->frame_filter[ANOTC_FRAME_LOG_STRING] = 1;
	opts->frame_filter_count = 1;
	return 0;
}

static int find_or_add_column(struct frame_definitions *defs, const char *name)
{
	for (int i = 0; i < defs->column_count; i++) {
		if (strcmp(defs->columns[i], name) == 0) {
			return i;
		}
	}

	if (defs->column_count >= MAX_DATA_COLUMNS) {
		fprintf(stderr, "Too many CSV columns in frame definitions.\n");
		return -1;
	}

	strncpy(defs->columns[defs->column_count], name, sizeof(defs->columns[defs->column_count]) - 1);
	defs->columns[defs->column_count][sizeof(defs->columns[defs->column_count]) - 1] = '\0';
	defs->column_count++;
	return defs->column_count - 1;
}

static int should_receive_frame(const struct options *opts, uint8_t frame_id);

static int prepare_definition_columns(struct frame_definitions *defs, const struct options *opts)
{
	if (!defs->enabled) {
		return 0;
	}

	for (int id = 0; id < 256; id++) {
		struct frame_def *frame = &defs->frames[id];

		if (!frame->present || !should_receive_frame(opts, (uint8_t)id)) {
			continue;
		}

		for (int i = 0; i < frame->param_count; i++) {
			int column_index = find_or_add_column(defs, frame->params[i].name);

			if (column_index < 0) {
				return -1;
			}
			frame->params[i].column_index = column_index;
		}
	}

	if (defs->column_count == 0) {
		fprintf(stderr, "No selected frame IDs exist in the frame definition file.\n");
		return -1;
	}

	return 0;
}

static int map_baud_rate(int baud_rate, speed_t *speed)
{
	switch (baud_rate) {
#ifdef B9600
	case 9600:
		*speed = B9600;
		return 0;
#endif
#ifdef B19200
	case 19200:
		*speed = B19200;
		return 0;
#endif
#ifdef B38400
	case 38400:
		*speed = B38400;
		return 0;
#endif
#ifdef B57600
	case 57600:
		*speed = B57600;
		return 0;
#endif
#ifdef B115200
	case 115200:
		*speed = B115200;
		return 0;
#endif
#ifdef B230400
	case 230400:
		*speed = B230400;
		return 0;
#endif
#ifdef B460800
	case 460800:
		*speed = B460800;
		return 0;
#endif
#ifdef B921600
	case 921600:
		*speed = B921600;
		return 0;
#endif
	default:
		return -1;
	}
}

static int configure_serial_port(int fd, int baud_rate)
{
	struct termios tty;
	speed_t speed = 0;
	int has_standard_baud = 0;

	if (tcgetattr(fd, &tty) != 0) {
		perror("tcgetattr");
		return -1;
	}

	has_standard_baud = (map_baud_rate(baud_rate, &speed) == 0);
	if (!has_standard_baud) {
#ifdef __APPLE__
		speed = B9600;
#else
		fprintf(stderr, "Unsupported baud rate on this platform: %d\n", baud_rate);
		return -1;
#endif
	}

	cfmakeraw(&tty);
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
#ifdef CRTSCTS
	tty.c_cflag &= ~CRTSCTS;
#endif
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_oflag = 0;
	tty.c_lflag = 0;
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 1;

	if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
		perror("cfsetispeed/cfsetospeed");
		return -1;
	}

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		perror("tcsetattr");
		return -1;
	}

#ifdef __APPLE__
	if (!has_standard_baud) {
		speed_t requested_speed = (speed_t)baud_rate;
		if (ioctl(fd, IOSSIOSPEED, &requested_speed) != 0) {
			perror("ioctl(IOSSIOSPEED)");
			return -1;
		}
	}
#endif

	if (tcflush(fd, TCIOFLUSH) != 0) {
		perror("tcflush");
		return -1;
	}

	return 0;
}

static int open_serial_port(const char *device_path, int baud_rate)
{
	int fd = open(device_path, O_RDWR | O_NOCTTY);

	if (fd < 0) {
		perror(device_path);
		return -1;
	}

	if (configure_serial_port(fd, baud_rate) != 0) {
		close(fd);
		return -1;
	}

	return fd;
}

static void anotc_add_checksum(uint8_t *frame, size_t frame_len_without_checksum)
{
	uint8_t sum_check = 0;
	uint8_t add_check = 0;

	for (size_t i = 0; i < frame_len_without_checksum; i++) {
		sum_check = (uint8_t)(sum_check + frame[i]);
		add_check = (uint8_t)(add_check + sum_check);
	}

	frame[frame_len_without_checksum] = sum_check;
	frame[frame_len_without_checksum + 1] = add_check;
}

static int write_all_fd(int fd, const uint8_t *buffer, size_t len)
{
	size_t written = 0;

	while (written < len) {
		ssize_t result = write(fd, buffer + written, len - written);
		if (result < 0) {
			if (errno == EINTR) {
				continue;
			}
			perror("write");
			return -1;
		}
		if (result == 0) {
			fprintf(stderr, "write returned 0 while sending ANOTC connect frame.\n");
			return -1;
		}
		written += (size_t)result;
	}

	return 0;
}

static int send_device_info_request(int fd, int verbose)
{
	uint8_t frame[ANOTC_V8_HEAD_SIZE + 3 + 2];
	size_t frame_len_without_checksum = ANOTC_V8_HEAD_SIZE + 3;

	frame[0] = ANOTC_V8_HEAD;
	frame[1] = ANOTC_PC_ADDR;
	frame[2] = ANOTC_DEVICE_ADDR;
	frame[3] = ANOTC_FRAME_CONFIG_CMD;
	frame[4] = 3;
	frame[5] = 0;
	frame[6] = ANOTC_CONFIG_FRAME_CMD_DEVICE_INFO;
	frame[7] = 0;
	frame[8] = 0;
	anotc_add_checksum(frame, frame_len_without_checksum);

	if (verbose) {
		debug_print_hex("TX connect request: ", frame, sizeof(frame));
	}
	if (write_all_fd(fd, frame, sizeof(frame)) != 0) {
		return -1;
	}
	if (tcdrain(fd) != 0) {
		perror("tcdrain");
		return -1;
	}

	return 0;
}

static void parser_reset_frame(struct parser *parser)
{
	parser->state = STATE_HEAD;
	parser->stored_len = 0;
	parser->expected_len = 0;
	parser->data_read = 0;
	parser->expected_sum = 0;
	parser->expected_add = 0;
	parser->computed_sum = 0;
	parser->computed_add = 0;
}

static void parser_init(struct parser *parser)
{
	memset(parser, 0, sizeof(*parser));
	parser_reset_frame(parser);
}

static void parser_store_byte(struct parser *parser, uint8_t value)
{
	if (parser->stored_len < sizeof(parser->frame)) {
		parser->frame[parser->stored_len] = value;
	}
	parser->stored_len++;
}

static void parser_update_checksum(struct parser *parser, uint8_t value)
{
	parser->computed_sum = (uint8_t)(parser->computed_sum + value);
	parser->computed_add = (uint8_t)(parser->computed_add + parser->computed_sum);
}

static int writer_emit(FILE *stream, const void *buffer, size_t len)
{
	if (len == 0) {
		return 0;
	}

	if (fwrite(buffer, 1, len, stream) != len) {
		perror("fwrite");
		return -1;
	}

	return 0;
}

static int build_timestamp(char *timestamp, size_t timestamp_size)
{
	struct timeval now;
	struct tm local_time;
	size_t count = 0;

	if (gettimeofday(&now, NULL) != 0) {
		perror("gettimeofday");
		return -1;
	}

	if (localtime_r(&now.tv_sec, &local_time) == NULL) {
		perror("localtime_r");
		return -1;
	}

	count = strftime(timestamp, timestamp_size, "%Y-%m-%d %H:%M:%S", &local_time);
	if (count == 0) {
		fprintf(stderr, "Failed to format timestamp.\n");
		return -1;
	}

	snprintf(timestamp + count, timestamp_size - count, ".%03d", (int)(now.tv_usec / 1000));
	return 0;
}

static uint64_t now_millis(void)
{
	struct timeval now;

	if (gettimeofday(&now, NULL) != 0) {
		return 0;
	}

	return (uint64_t)now.tv_sec * 1000ULL + (uint64_t)(now.tv_usec / 1000);
}

static int writer_emit_csv_field(FILE *stream, const char *text, size_t len)
{
	if (writer_emit(stream, "\"", 1) != 0) {
		return -1;
	}

	for (size_t i = 0; i < len; i++) {
		if (text[i] == '"') {
			if (writer_emit(stream, "\"\"", 2) != 0) {
				return -1;
			}
		} else {
			if (writer_emit(stream, text + i, 1) != 0) {
				return -1;
			}
		}
	}

	return writer_emit(stream, "\"", 1);
}

static void hex_encode(const uint8_t *data, size_t len, char *hex, size_t hex_size)
{
	static const char digits[] = "0123456789ABCDEF";
	size_t needed = len * 2 + 1;

	if (hex_size == 0) {
		return;
	}
	if (needed > hex_size) {
		len = (hex_size - 1) / 2;
	}

	for (size_t i = 0; i < len; i++) {
		hex[i * 2] = digits[(data[i] >> 4) & 0x0F];
		hex[i * 2 + 1] = digits[data[i] & 0x0F];
	}
	hex[len * 2] = '\0';
}

static void debug_print_hex(const char *prefix, const uint8_t *data, size_t len)
{
	size_t display_len = len > 64 ? 64 : len;

	fprintf(stderr, "%s", prefix);
	for (size_t i = 0; i < display_len; i++) {
		fprintf(stderr, "%s%02X", i == 0 ? "" : " ", data[i]);
	}
	if (display_len < len) {
		fprintf(stderr, " ...");
	}
	fprintf(stderr, "\n");
}

static size_t trimmed_text_len(const uint8_t *text, size_t len)
{
	while (len > 0 && (text[len - 1] == '\n' || text[len - 1] == '\r')) {
		len--;
	}

	return len;
}

static int writer_write_csv_row_to_stream(FILE *stream, const char *timestamp, const char *frame_id, const uint8_t *message, size_t message_len, const char *payload_hex)
{
	message_len = trimmed_text_len(message, message_len);

	if (writer_emit_csv_field(stream, timestamp, strlen(timestamp)) != 0 ||
	    writer_emit(stream, ",", 1) != 0 ||
	    writer_emit_csv_field(stream, frame_id, strlen(frame_id)) != 0 ||
	    writer_emit(stream, ",", 1) != 0 ||
	    writer_emit_csv_field(stream, (const char *)message, message_len) != 0 ||
	    writer_emit(stream, ",", 1) != 0 ||
	    writer_emit_csv_field(stream, payload_hex, strlen(payload_hex)) != 0 ||
	    writer_emit(stream, "\n", 1) != 0) {
		return -1;
	}

	return 0;
}

static int writer_write_frame_csv_row(struct writer *writer, uint8_t frame_id, const uint8_t *payload, size_t payload_len, const uint8_t *message, size_t message_len)
{
	char timestamp[32];
	char frame_id_text[8];
	char payload_hex[ANOTC_DATA_MAX_SIZE * 2 + 1];

	if (build_timestamp(timestamp, sizeof(timestamp)) != 0) {
		return -1;
	}

	snprintf(frame_id_text, sizeof(frame_id_text), "0x%02X", frame_id);
	hex_encode(payload, payload_len, payload_hex, sizeof(payload_hex));

	if (writer_write_csv_row_to_stream(writer->file, timestamp, frame_id_text, message, message_len, payload_hex) != 0) {
		return -1;
	}
	if (writer->echo_stdout && writer_write_csv_row_to_stream(stdout, timestamp, frame_id_text, message, message_len, payload_hex) != 0) {
		return -1;
	}

	if (fflush(writer->file) != 0) {
		perror("fflush");
		return -1;
	}
	if (writer->echo_stdout && fflush(stdout) != 0) {
		perror("fflush");
		return -1;
	}

	return 0;
}

static int writer_write_header_to_stream(FILE *stream, const struct frame_definitions *defs)
{
	if (!defs->enabled) {
		return writer_emit(stream, CSV_HEADER, strlen(CSV_HEADER));
	}

	if (writer_emit(stream, "timestamp", strlen("timestamp")) != 0) {
		return -1;
	}
	for (int i = 0; i < defs->column_count; i++) {
		if (writer_emit(stream, ",", 1) != 0 ||
		    writer_emit_csv_field(stream, defs->columns[i], strlen(defs->columns[i])) != 0) {
			return -1;
		}
	}

	return writer_emit(stream, "\n", 1);
}

static uint16_t read_u16_le(const uint8_t *data)
{
	return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *data)
{
	return (uint32_t)data[0] |
	       ((uint32_t)data[1] << 8) |
	       ((uint32_t)data[2] << 16) |
	       ((uint32_t)data[3] << 24);
}

static uint64_t read_u64_le(const uint8_t *data)
{
	uint64_t value = 0;

	for (int i = 0; i < 8; i++) {
		value |= (uint64_t)data[i] << (i * 8);
	}

	return value;
}

static void format_scaled_signed(int64_t raw, double scale, char *out, size_t out_size)
{
	if (scale == 1.0) {
		snprintf(out, out_size, "%" PRId64, raw);
	} else {
		snprintf(out, out_size, "%.9g", (double)raw * scale);
	}
}

static void format_scaled_unsigned(uint64_t raw, double scale, char *out, size_t out_size)
{
	if (scale == 1.0) {
		snprintf(out, out_size, "%" PRIu64, raw);
	} else {
		snprintf(out, out_size, "%.9g", (double)raw * scale);
	}
}

static int format_param_value(const struct frame_param_def *param, const uint8_t *payload, size_t payload_len, size_t *offset, char *out, size_t out_size)
{
	uint32_t raw32 = 0;
	uint64_t raw64 = 0;
	float float_value = 0.0f;
	double double_value = 0.0;

	out[0] = '\0';
	if (*offset + param->size > payload_len) {
		*offset = payload_len;
		return 0;
	}

	switch (param->type) {
	case FIELD_INT8:
		format_scaled_signed((int8_t)payload[*offset], param->scale, out, out_size);
		break;
	case FIELD_UINT8:
		format_scaled_unsigned(payload[*offset], param->scale, out, out_size);
		break;
	case FIELD_INT16:
		format_scaled_signed((int16_t)read_u16_le(payload + *offset), param->scale, out, out_size);
		break;
	case FIELD_UINT16:
		format_scaled_unsigned(read_u16_le(payload + *offset), param->scale, out, out_size);
		break;
	case FIELD_INT32:
		format_scaled_signed((int32_t)read_u32_le(payload + *offset), param->scale, out, out_size);
		break;
	case FIELD_UINT32:
		format_scaled_unsigned(read_u32_le(payload + *offset), param->scale, out, out_size);
		break;
	case FIELD_INT64:
		format_scaled_signed((int64_t)read_u64_le(payload + *offset), param->scale, out, out_size);
		break;
	case FIELD_UINT64:
		format_scaled_unsigned(read_u64_le(payload + *offset), param->scale, out, out_size);
		break;
	case FIELD_FLOAT:
		raw32 = read_u32_le(payload + *offset);
		memcpy(&float_value, &raw32, sizeof(float_value));
		snprintf(out, out_size, "%.9g", (double)float_value * param->scale);
		break;
	case FIELD_DOUBLE:
		raw64 = read_u64_le(payload + *offset);
		memcpy(&double_value, &raw64, sizeof(double_value));
		snprintf(out, out_size, "%.17g", double_value * param->scale);
		break;
	}

	*offset += param->size;
	return 0;
}

static int writer_write_definition_row_to_stream(FILE *stream, const char *timestamp, char values[MAX_DATA_COLUMNS][64], int column_count)
{
	if (writer_emit_csv_field(stream, timestamp, strlen(timestamp)) != 0) {
		return -1;
	}
	for (int i = 0; i < column_count; i++) {
		if (writer_emit(stream, ",", 1) != 0 ||
		    writer_emit_csv_field(stream, values[i], strlen(values[i])) != 0) {
			return -1;
		}
	}

	return writer_emit(stream, "\n", 1);
}

static int writer_write_definition_csv_row(struct writer *writer, const struct frame_definitions *defs, uint8_t frame_id, const uint8_t *payload, size_t payload_len)
{
	const struct frame_def *frame = &defs->frames[frame_id];
	char timestamp[32];
	char values[MAX_DATA_COLUMNS][64];
	size_t offset = 0;

	if (!frame->present) {
		return 0;
	}

	memset(values, 0, sizeof(values));
	for (int i = 0; i < frame->param_count; i++) {
		const struct frame_param_def *param = &frame->params[i];

		if (param->column_index < 0 || param->column_index >= defs->column_count) {
			continue;
		}
		if (format_param_value(param, payload, payload_len, &offset, values[param->column_index], sizeof(values[param->column_index])) != 0) {
			return -1;
		}
	}

	if (build_timestamp(timestamp, sizeof(timestamp)) != 0) {
		return -1;
	}
	if (writer_write_definition_row_to_stream(writer->file, timestamp, values, defs->column_count) != 0) {
		return -1;
	}
	if (writer->echo_stdout && writer_write_definition_row_to_stream(stdout, timestamp, values, defs->column_count) != 0) {
		return -1;
	}
	if (fflush(writer->file) != 0) {
		perror("fflush");
		return -1;
	}
	if (writer->echo_stdout && fflush(stdout) != 0) {
		perror("fflush");
		return -1;
	}

	return 0;
}

static int writer_init(struct writer *writer, const struct options *opts, const struct frame_definitions *defs)
{
	struct stat output_stat;
	int should_write_header = 1;

	memset(writer, 0, sizeof(*writer));

	if (strcmp(opts->output_path, "-") == 0) {
		writer->file = stdout;
		writer->echo_stdout = 0;
		return writer_write_header_to_stream(writer->file, defs);
	}

	if (opts->append && stat(opts->output_path, &output_stat) == 0 && output_stat.st_size > 0) {
		should_write_header = 0;
	}

	writer->file = fopen(opts->output_path, opts->append ? "ab" : "wb");
	if (writer->file == NULL) {
		perror(opts->output_path);
		return -1;
	}

	writer->echo_stdout = opts->quiet ? 0 : 1;
	if (should_write_header) {
		if (writer_write_header_to_stream(writer->file, defs) != 0) {
			return -1;
		}
	}
	if (writer->echo_stdout) {
		if (writer_write_header_to_stream(stdout, defs) != 0) {
			return -1;
		}
	}

	return 0;
}

static void writer_close(struct writer *writer)
{
	if (writer->file != NULL && writer->file != stdout) {
		fclose(writer->file);
		writer->file = NULL;
	}
}

static int should_receive_frame(const struct options *opts, uint8_t frame_id)
{
	return opts->receive_all_frames || opts->frame_filter[frame_id];
}

static int handle_valid_frame(struct parser *parser, struct writer *writer, const struct options *opts, const struct frame_definitions *defs)
{
	const uint8_t *payload = parser->frame + ANOTC_V8_HEAD_SIZE;
	uint8_t function_id = parser->frame[3];
	const uint8_t empty_message[] = "";

	if (!should_receive_frame(opts, function_id)) {
		return 0;
	}

	if (defs->enabled) {
		if (!defs->frames[function_id].present) {
			if (opts->verbose) {
				fprintf(stderr, "Ignoring selected frame 0x%02X: no JSON definition.\n", function_id);
			}
			return 0;
		}
		parser->matched_frames++;
		if (opts->verbose) {
			fprintf(stderr, "RX frame 0x%02X len=%u matched JSON definition.\n", function_id, parser->expected_len);
		}
		return writer_write_definition_csv_row(writer, defs, function_id, payload, parser->expected_len);
	}

	parser->matched_frames++;
	if (opts->verbose) {
		fprintf(stderr, "RX frame 0x%02X len=%u matched filter.\n", function_id, parser->expected_len);
	}
	if (function_id == ANOTC_FRAME_LOG_STRING) {
		if (parser->expected_len <= 1) {
			return writer_write_frame_csv_row(writer, function_id, payload, parser->expected_len, empty_message, 0);
		}

		return writer_write_frame_csv_row(writer, function_id, payload, parser->expected_len, payload + 1, parser->expected_len - 1);
	}

	return writer_write_frame_csv_row(writer, function_id, payload, parser->expected_len, empty_message, 0);
}

static int parser_consume_byte(struct parser *parser, struct writer *writer, const struct options *opts, const struct frame_definitions *defs, uint8_t value)
{
	parser->bytes_received++;

	switch (parser->state) {
	case STATE_HEAD:
		if (value == ANOTC_V8_HEAD) {
			parser_reset_frame(parser);
			parser_store_byte(parser, value);
			parser_update_checksum(parser, value);
			parser->state = STATE_SRC_ADDR;
		}
		break;
	case STATE_SRC_ADDR:
		parser_store_byte(parser, value);
		parser_update_checksum(parser, value);
		parser->state = STATE_DST_ADDR;
		break;
	case STATE_DST_ADDR:
		parser_store_byte(parser, value);
		parser_update_checksum(parser, value);
		parser->state = STATE_FUNC;
		break;
	case STATE_FUNC:
		parser_store_byte(parser, value);
		parser_update_checksum(parser, value);
		parser->state = STATE_LEN1;
		break;
	case STATE_LEN1:
		parser_store_byte(parser, value);
		parser_update_checksum(parser, value);
		parser->expected_len = value;
		parser->state = STATE_LEN2;
		break;
	case STATE_LEN2:
		parser_store_byte(parser, value);
		parser_update_checksum(parser, value);
		parser->expected_len |= (uint16_t)value << 8;
		if (parser->expected_len > ANOTC_DATA_MAX_SIZE) {
			parser->oversize_frames++;
		}
		parser->state = (parser->expected_len == 0) ? STATE_SUM_CHECK : STATE_DATA;
		break;
	case STATE_DATA:
		if (parser->data_read < ANOTC_DATA_MAX_SIZE) {
			parser_store_byte(parser, value);
		}
		parser->data_read++;
		parser_update_checksum(parser, value);
		if (parser->data_read == parser->expected_len) {
			parser->state = STATE_SUM_CHECK;
		}
		break;
	case STATE_SUM_CHECK:
		parser->expected_sum = value;
		parser->state = STATE_ADD_CHECK;
		break;
	case STATE_ADD_CHECK:
		parser->expected_add = value;
		parser->total_frames++;
		if (parser->expected_len > ANOTC_DATA_MAX_SIZE) {
			parser_reset_frame(parser);
			break;
		}
		if (parser->expected_sum == parser->computed_sum &&
		    parser->expected_add == parser->computed_add) {
			parser->seen_frames[parser->frame[3]]++;
			if (handle_valid_frame(parser, writer, opts, defs) != 0) {
				return -1;
			}
		} else {
			parser->checksum_errors++;
			if (opts->verbose) {
				fprintf(
					stderr,
					"Checksum error for frame candidate 0x%02X len=%u expected=%02X/%02X computed=%02X/%02X\n",
					parser->frame[3],
					parser->expected_len,
					parser->expected_sum,
					parser->expected_add,
					parser->computed_sum,
					parser->computed_add
				);
			}
		}
		parser_reset_frame(parser);
		break;
	}

	return 0;
}

int main(int argc, char **argv)
{
	struct options opts;
	struct writer writer;
	struct parser parser;
	struct frame_definitions defs;
	uint8_t read_buffer[256];
	int serial_fd = -1;
	int parse_result = 0;
	ssize_t bytes_read = 0;
	uint64_t next_connect_request_ms = 0;

	parse_result = parse_options(argc, argv, &opts);
	if (parse_result > 0) {
		return 0;
	}
	if (parse_result < 0) {
		return 1;
	}

	if (load_frame_definitions(opts.frame_def_path, &defs) != 0) {
		return 1;
	}
	if (ensure_frame_filters(&opts, &defs) != 0 ||
	    prepare_definition_columns(&defs, &opts) != 0) {
		return 1;
	}

	if (writer_init(&writer, &opts, &defs) != 0) {
		return 1;
	}

	serial_fd = open_serial_port(opts.device_path, opts.baud_rate);
	if (serial_fd < 0) {
		writer_close(&writer);
		return 1;
	}
	if (opts.verbose) {
		fprintf(stderr, "Opened serial port %s at %d baud.\n", opts.device_path, opts.baud_rate);
	}
	if (!opts.no_connect) {
		if (send_device_info_request(serial_fd, opts.verbose) != 0) {
			close(serial_fd);
			writer_close(&writer);
			return 1;
		}
		next_connect_request_ms = now_millis() + 1000ULL;
	} else if (opts.verbose) {
		fprintf(stderr, "ANOTC connect request disabled by --no-connect.\n");
	}

	if (install_signal_handlers() != 0) {
		close(serial_fd);
		writer_close(&writer);
		return 1;
	}

	parser_init(&parser);

	fprintf(
		stderr,
		"Listening on %s at %d baud, writing decoded ANOTC frames to %s\n",
		opts.device_path,
		opts.baud_rate,
		strcmp(opts.output_path, "-") == 0 ? "stdout" : opts.output_path
	);

	while (!g_should_stop) {
		uint64_t current_ms = now_millis();

		if (!opts.no_connect && parser.total_frames == 0 && current_ms != 0 && current_ms >= next_connect_request_ms) {
			if (send_device_info_request(serial_fd, opts.verbose) != 0) {
				close(serial_fd);
				writer_close(&writer);
				return 1;
			}
			next_connect_request_ms = current_ms + 1000ULL;
		}

		bytes_read = read(serial_fd, read_buffer, sizeof(read_buffer));
		if (bytes_read < 0) {
			if (errno == EINTR) {
				continue;
			}
			perror("read");
			close(serial_fd);
			writer_close(&writer);
			return 1;
		}

		if (bytes_read == 0) {
			continue;
		}

		if (opts.verbose) {
			char prefix[64];

			snprintf(prefix, sizeof(prefix), "RX raw %zd bytes: ", bytes_read);
			debug_print_hex(prefix, read_buffer, (size_t)bytes_read);
		}
		for (ssize_t i = 0; i < bytes_read; i++) {
			if (parser_consume_byte(&parser, &writer, &opts, &defs, read_buffer[i]) != 0) {
				close(serial_fd);
				writer_close(&writer);
				return 1;
			}
		}
	}

	close(serial_fd);
	writer_close(&writer);

	fprintf(
		stderr,
		"\nStopped. bytes=%llu frames=%llu matched_frames=%llu checksum_errors=%llu oversize_frames=%llu\n",
		(unsigned long long)parser.bytes_received,
		(unsigned long long)parser.total_frames,
		(unsigned long long)parser.matched_frames,
		(unsigned long long)parser.checksum_errors,
		(unsigned long long)parser.oversize_frames
	);
	fprintf(stderr, "Seen valid frame IDs:");
	int seen_any_frame = 0;
	for (int i = 0; i < 256; i++) {
		if (parser.seen_frames[i] > 0) {
			fprintf(stderr, " 0x%02X(%llu)", i, (unsigned long long)parser.seen_frames[i]);
			seen_any_frame = 1;
		}
	}
	if (!seen_any_frame) {
		fprintf(stderr, " none");
	}
	fprintf(stderr, "\n");

	return 0;
}
