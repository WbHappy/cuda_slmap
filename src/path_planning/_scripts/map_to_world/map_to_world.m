clear all;
close all;
clc;

pkg load symbolic;


syms WX     %World position x
syms WY     %World position y
syms MX     %Map position x
syms MY     %Map position y

syms mo     %Map orinetation
syms ms     %Map scale
syms mf     %Map offset pixels
syms po     %Point orientation
syms pd     %Point distance

atan(WY/WY) 
