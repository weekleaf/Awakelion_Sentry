uint8 gamestart
uint8 robot_id  #机器人id
float32 we_x  #哨兵x
float32 we_y  #哨兵y
float32 we_angle  #哨兵angle
uint8 armor_id  #不是装甲板受击时就为0，是的话就是装甲板id
uint8 HP_deduction_reason  #装甲板受击就是0，其他就是其他的扣血原因
uint16 we_outpost_HP
uint16 enemy_outpost_HP
uint16 we_base_HP
uint16 enemy_base_HP
uint8 game_progress
uint16 stage_remain_time