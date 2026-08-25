To load the pretrained GST checkpoints in this folder, change `pred.model_dir` in `crowd_nav/configs/config.py`:
- If the human attributes and goals are not randomized, set to 'gst_default' (originally '100-gumbel_social_transformer-faster_lstm-lr_0.001-init_temp_0.5-edge_head_0-ebd_64-snl_1-snh_8-seed_1000');
- Otherwise, set to 'gst_rand' (originally '100-gumbel_social_transformer-faster_lstm-lr_0.001-init_temp_0.5-edge_head_0-ebd_64-snl_1-snh_8-seed_1000_rand').

To train a GST model by yourself, refer to https://github.com/tedhuang96/gst. 