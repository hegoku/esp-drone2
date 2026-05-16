#ifndef DATA_ANALYSIS_H
#define DATA_ANALYSIS_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum DATA_ANALYSIS_ITEM_TYPE
{
	DATA_TYPE_UINT8,
	DATA_TYPE_INT8,
	DATA_TYPE_UINT16,
	DATA_TYPE_INT16,
	DATA_TYPE_UINT32,
	DATA_TYPE_INT32,
	DATA_TYPE_UINT64,
	DATA_TYPE_INT64,
	DATA_TYPE_FLOAT,
	DATA_TYPE_DOUBLE,
	DATA_TYPE_STRING
};

struct data_analysis_item {
	enum DATA_ANALYSIS_ITEM_TYPE type;
	char name[20];
	char *info;
	void *variable;
};

#define DA_CONCAT_(a, b) a##b
#define DA_CONCAT(a, b) ANOTC_CONCAT_(a, b)

#define DA_ADD_ITEM(type_, name_, info_, value_pointer_) \
    DA_ADD_ITEM_IMPL(__COUNTER__, type_, name_, info_, value_pointer_)

#define DA_ADD_ITEM_IMPL(n, par_id_, type_, name_, par_info_, value_pointer_)        \
    static const struct data_analysis_item DA_CONCAT_(__data_analysis_, n) = {               \
        .type = (type_),                                                                  \
        .name = (name_),                                                          \
        .info = (par_info_),                                                          \
        .variable = value_pointer_,                                             \
    };                                                                                    \
    static const struct data_analysis_item * const DA_CONCAT_(__data_analysis_ptr_, n)       \
        __attribute__((used, section(".data_analysis_item_ptr"))) = &DA_CONCAT_(__data_analysis_, n)

extern const struct data_analysis_item * const _data_analysis_item_start[];
extern const struct data_analysis_item * const _data_analysis_item_end[];

static inline unsigned short data_analysis_item_count()
{
    return (unsigned short)(_data_analysis_item_end - _data_analysis_item_start);
}

static inline const struct data_analysis_item *da_item_at(unsigned short index)
{
    return _data_analysis_item_start[index];
}

static inline const struct data_analysis_item *da_find_by_name(const char *name)
{
    if (name == NULL) {
        return NULL;
    }

    for (int i = 0; i < data_analysis_item_count(); i++) {
        const struct data_analysis_item *item = _data_analysis_item_start[i];

        if (item == NULL) {
            continue;
        }

        if (strcmp(item->name, name) == 0) {
            return item;
        }
    }

    return NULL;
}

static inline size_t da_find_index_by_name(const char *name)
{
    if (name == NULL) {
        return -1;
    }

    for (size_t i = 0; i < data_analysis_item_count(); i++) {
        const struct data_analysis_item *item = _data_analysis_item_start[i];

        if (item == NULL) {
            continue;
        }

        if (strcmp(item->name, name) == 0) {
            return i;
        }
    }

    return -1;
}

#endif