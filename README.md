# Delivery-Bot
Code for an autonomous delivery bot created for my senior project. The delivery bot is controlled by a jetson nano and an arduino nano.

The Arduino sketch now exposes only the motor and encoder commands used by the
Jetson drive code. Robot dimensions, motor pins, and PID/feed-forward tuning
values live in `Jetson/config.py`.
