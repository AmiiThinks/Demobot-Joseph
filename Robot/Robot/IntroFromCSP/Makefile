all: sarsa-remote random-policy


sarsa-remote: sarsa-remote.c baseline-v97.c
	g++ -pthread -lm -O2 -Wall sarsa-remote.c baseline-v97.c -o sarsa-remote

random-policy: random-policy.c baseline-v97.c
	g++ -pthread -lm -O2 -Wall random-policy.c baseline-v97.c -o random-policy