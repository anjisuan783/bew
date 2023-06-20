#include "flex_fec_sender.h"
#include "flex_fec_receiver.h"
#include <assert.h>

#define data_info "1234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890"
#define segs_num 12

void test_fec_xor()
{
	sim_segment_t *segs[segs_num], *seg;
	sim_fec_t fec;

	sim_segment_t* dst_segs[segs_num], recover_seg;

	uint32_t i, count, fec_id;

	for (i = 0; i < segs_num; i++){
		seg = calloc(1, sizeof(sim_segment_t));
		seg->packet_id = seg->fid = i;
		seg->payload_type = i % 2;
		seg->ftype = 1;
		seg->index = 0;
		seg->total = 1;
		seg->timestamp = i * 100;

		seg->data_size = strlen(data_info) - segs_num + i + 1;
		memcpy(seg->data, data_info, seg->data_size);

		segs[i] = seg;
	}

	if (flex_fec_generate(segs, segs_num, &fec) != 0){
		printf("fec generate failed!\n");
		goto free_segments;
	}

	if (fec.fec_data_size > strlen(data_info)){
		printf("fec data size is error, fec data size = %d, max data size = %d\n", fec.fec_data_size, strlen(data_info));
		goto free_segments;
	}

	count = 0;
	fec_id = 9;
	for (i = 0; i < segs_num; i++){
		if (i != fec_id)
			dst_segs[count++] = segs[i];
	}

	if (flex_fec_recover(dst_segs, count, &fec, &recover_seg) != 0){
		printf("fec recover failed!\n");
		goto free_segments;
	}
	
	recover_seg.data[seg->data_size] = 0;

	printf("seg[%d]:\n", fec_id);
	printf("\tpacket id = %d\n", recover_seg.packet_id);
	printf("\tfid = %d\n", recover_seg.fid);
	printf("\ttimestamp = %d\n", recover_seg.timestamp);
	printf("\tpayload_type = %d\n", recover_seg.payload_type);
	printf("\tftype = %d\n", recover_seg.ftype);
	printf("\ttotal = %d\n", recover_seg.total);
	printf("\tindex = %d\n", recover_seg.index);
	printf("\tdata size = %d\n", recover_seg.data_size);
	printf("\tdata info = %s\n", (char*)recover_seg.data);

free_segments:
	for (i = 0; i < segs_num; i++){
		free(segs[i]);
	}
}

static const uint16_t segments_num[10] = {1, 5, 6, 15, 20, 36, 41, 50, 72, 122};
static const uint8_t protection_fact[13] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 52, 128, 255};
void test_num_fec()
{
	flex_fec_sender_t fec;
	fec.col = 0;
	fec.row = 0;

	int rc = 0;

	for (int i = 0; i < sizeof(segments_num)/sizeof(uint16_t); ++i) {
		printf("case %d segment cout = %d \n", i+1, segments_num[i]);
		fec.segs_count = segments_num[i];
		for (int j = 0; j < sizeof(protection_fact); ++j) {
			rc = flex_fec_sender_num_packets(&fec, protection_fact[j]);
			printf("proctection=%d, col=%d, row=%d %s\n", protection_fact[j], fec.col, fec.row, rc?"fec on":"fec off");
		}
	}
}

#define FLEX_SEG_NUM 21

static void segment_assert(sim_segment_t* src, sim_segment_t* dst)
{
	assert(src->packet_id == dst->packet_id);
	assert(src->payload_type == dst->payload_type);
	assert(src->fid == dst->fid);
	assert(src->timestamp == dst->timestamp);
	assert(src->ftype == dst->ftype);
	assert(src->index == dst->index);
	assert(src->total == dst->total);

	assert(src->data_size == dst->data_size);
	assert(memcmp(src->data, dst->data, dst->data_size) == 0);
}

static void verify_flex_sender(sim_segment_t* segments[], int segs_count, base_list_t* l)
{
	base_list_unit_t* iter;
	sim_fec_t* fec_packet;

	sim_segment_t* segs[100], seg;
	int i, index, count, pos;

	LIST_FOREACH(l, iter){
		fec_packet = iter->pdata;

		index = fec_packet->index & 0x7f;
		count = 0;

		if ((fec_packet->index & 0x80) == 0){ /*validate row*/
			for (i = 0; i < fec_packet->col; i++){
			pos = index * fec_packet->col + i;
			if (i > 0 && pos < segs_count)
				segs[count++] = segments[pos];
			}

			if (flex_fec_recover(segs, count, fec_packet, &seg) == 0)
				segment_assert(segments[index * fec_packet->col], &seg);
			else
				printf("recover failed, row fec index = %d\n", index);
		} else {/* validate column */
			for (i = 0; i < fec_packet->row; i++){
				pos = i * fec_packet->col + index;
				if (i > 0 && pos < segs_count)
					segs[count++] = segments[pos];
			}

			if (flex_fec_recover(segs, count, fec_packet, &seg) == 0)
				segment_assert(segments[index], &seg);
			else
				printf("recover failed, colum fec index = %d\n", index);
		}
	}
}

void test_flex_sender(uint8_t protect_fraction)
{
	sim_segment_t *segs[FLEX_SEG_NUM], *seg;
	flex_fec_sender_t* flex = flex_fec_sender_create();

	for (int i = 0; i < FLEX_SEG_NUM; i++){
		seg = calloc(1, sizeof(sim_segment_t));
		seg->packet_id = seg->fid = i;
		seg->payload_type = i % 2;
		seg->ftype = 1;
		seg->index = 0;
		seg->total = 1;
		seg->timestamp = i * 100;

		seg->data_size = strlen(data_info) - FLEX_SEG_NUM + i + 1;
		memcpy(seg->data, data_info, seg->data_size);
		segs[i] = seg;
		flex_fec_sender_add_segment(flex, seg);
	}

	base_list_t* fec_list = create_list();
	// single row fec proctection
	flex_fec_sender_update(flex, protect_fraction, fec_list);

	printf("%s, segment count = %d, protect = %u, fec num = %d\n", 
		protect_fraction > 52 ? "multiFEC":"singleFEC", 
		FLEX_SEG_NUM, 
		protect_fraction, 
		list_size(fec_list));
		
	if (list_size(fec_list) > 0){
		sim_fec_t* fec_packet = list_front(fec_list);
		printf("fec col = %d, row = %d\n", fec_packet->col, fec_packet->row);
		verify_flex_sender(segs, FLEX_SEG_NUM, fec_list);
	}

	flex_fec_sender_release(flex, fec_list);

	destroy_list(fec_list);
	flex_fec_sender_destroy(flex);

	for (int i = 0; i < FLEX_SEG_NUM; i++){
		free(segs[i]);
	}
}

#define RECV_SEG_NUM 25

static void packet_add_recover_list(skiplist_t *l, sim_segment_t* seg)
{
	skiplist_item_t key,val;

	if (seg == NULL)
		return;

	key.u32 = seg->packet_id;
	if (skiplist_search(l, key) == NULL){
		key.u32 = seg->packet_id;
		val.ptr = seg;
		skiplist_insert(l, key, val);
	}
	else
		free(seg);
}

static void test_free_rec_item(skiplist_item_t key, skiplist_item_t val, void* args)
{
	sim_segment_t* seg;
	seg = val.ptr;
	if (seg != NULL)
		free(seg);
}

#define log_buffer_len  1024
static char log_buffer[log_buffer_len];
static int log_buffer_pos = 0;

static int test_flex_order(sim_segment_t* segs[], base_list_t* fec_list, uint8_t loss_arr[], int loss_num)
{
	skiplist_t* rec_id_verify_list = skiplist_create(idu32_compare, NULL, NULL);
	log_buffer_pos += snprintf(log_buffer + log_buffer_pos, log_buffer_len-log_buffer_pos, " lost id[");
	for (int i = 0; i < loss_num; ++i) {
		skiplist_item_t key, val;
		key.u32 = (uint32_t)loss_arr[i];
		val.ptr = NULL;
		skiplist_insert(rec_id_verify_list, key, val);
		log_buffer_pos += snprintf(log_buffer + log_buffer_pos, log_buffer_len-log_buffer_pos, "%u,", (uint32_t)loss_arr[i]);
	}
	log_buffer_pos += snprintf(log_buffer + log_buffer_pos, log_buffer_len-log_buffer_pos, "]");

	flex_fec_receiver_t* receiver = flex_fec_receiver_create(NULL, NULL, NULL);
	sim_fec_t* fec_packet = list_front(fec_list);
	flex_fec_receiver_active(receiver, fec_packet->fec_id, fec_packet->col, fec_packet->row, fec_packet->base_id, fec_packet->count);
	log_buffer_pos += snprintf(log_buffer + log_buffer_pos, log_buffer_len-log_buffer_pos, ", fec row = %d, colum = %d\n", fec_packet->row, fec_packet->col);

	base_list_t *out = create_list();  //recovered segments list
	for (int i = 0; i < RECV_SEG_NUM; i++){
		int flag = 0;
		for (int j = 0; j < loss_num; j++){
			if (loss_arr[j] == i)
				flag = 1;
		}
		if (flag == 0){
			flex_fec_receiver_on_segment(receiver, segs[i], out);
			assert(list_size(out) == 0);
		}
	}
	
	skiplist_t* rec_map = skiplist_create(idu32_compare, NULL, NULL); //recovered fec segments list
	base_list_unit_t* iter;
	LIST_FOREACH(fec_list, iter){
		packet_add_recover_list(rec_map, flex_fec_receiver_on_fec(receiver, iter->pdata));
	}

	sim_segment_t* seg;
	skiplist_iter_t* sl_iter;
	while (skiplist_size(rec_map) > 0){
		sl_iter = skiplist_first(rec_map);
		seg = sl_iter->val.ptr;

		log_buffer_pos += snprintf(log_buffer + log_buffer_pos, log_buffer_len-log_buffer_pos, ", recover id=%u", seg->packet_id);
		segment_assert(seg, segs[seg->packet_id]);

	// check recovered meta segment in lost array
		int found = 0;
		for(int i = 0; i < loss_num; ++i) {
			if (loss_arr[i] == seg->packet_id) {
				found = 1;
				skiplist_item_t key;
				key.u32 = (uint32_t)seg->packet_id;
				skiplist_remove(rec_id_verify_list, key);
				break;
			}
		}
		if (!found) {
			log_buffer_pos += snprintf(log_buffer + log_buffer_pos, log_buffer_len-log_buffer_pos, " recover fec segment not found, id=%u\n", seg->packet_id);
			return -1;
		}

		flex_fec_receiver_on_segment(receiver, segs[seg->packet_id], out);
		
		skiplist_remove(rec_map, sl_iter->key);

		do{
			packet_add_recover_list(rec_map, list_pop(out));  // recover matrix
		} while (list_size(out) > 0);
	}
	destroy_list(out);
	skiplist_destroy(rec_map);

	flex_fec_receiver_desotry(receiver);

	int ret = skiplist_size(rec_id_verify_list);
	
	if (ret != 0) {
		log_buffer_pos += snprintf(log_buffer + log_buffer_pos, log_buffer_len-log_buffer_pos, " recover meta failed. left[");
		skiplist_iter_t* it;
		SKIPLIST_FOREACH(rec_id_verify_list, it) {
			log_buffer_pos += snprintf(log_buffer + log_buffer_pos, log_buffer_len-log_buffer_pos, "%u,", it->key.u32);
		}
		log_buffer_pos += snprintf(log_buffer + log_buffer_pos, log_buffer_len-log_buffer_pos, "]");
	}
	skiplist_destroy(rec_id_verify_list);
	log_buffer_pos = snprintf(log_buffer + log_buffer_pos, log_buffer_len-log_buffer_pos, "\n");
	return ret;
}

//static uint8_t loss[] = { 5, 6, 7, 8, 9 };

int test_flex_receiver(uint8_t protect_fraction, uint8_t loss_num, uint8_t loss[])
{
	sim_segment_t *segs[RECV_SEG_NUM];

	flex_fec_sender_t* sender = flex_fec_sender_create();

	// generate test data
	for (int i = 0; i < RECV_SEG_NUM; i++){
		sim_segment_t* seg = calloc(1, sizeof(sim_segment_t));
		seg->packet_id = seg->fid = i;
		seg->payload_type = i % 2;
		seg->ftype = 1;
		seg->index = 0;
		seg->total = 1;
		seg->timestamp = i * 100;

		seg->data_size = strlen(data_info) - RECV_SEG_NUM + i + 1;
		memcpy(seg->data, data_info, seg->data_size);
		segs[i] = seg;
		flex_fec_sender_add_segment(sender, seg);
	}

	base_list_t* fec_list = create_list();
	flex_fec_sender_update(sender, protect_fraction, fec_list);
	log_buffer_pos = snprintf(log_buffer, 1023, "protect = %u, %s, segment count = %d,  fec num = %d, lost count = %d", 
		protect_fraction, (protect_fraction >= 10 && RECV_SEG_NUM > 5)? "multiFEC" : "singleFEC", 
		FLEX_SEG_NUM, list_size(fec_list), loss_num);

	if (list_size(fec_list) > 0){
		if (test_flex_order(segs, fec_list, loss, SU_MIN(loss_num, RECV_SEG_NUM)) != 0) {
			printf("%s", log_buffer);
			return -1;
		}
	}

	for (int i = 0; i < RECV_SEG_NUM; ++i)
		free(segs[i]);

	destroy_list(fec_list);
	flex_fec_sender_destroy(sender);
	return 0;
}

