.SUFFIXES: .c .o
CC = gcc
EXEC = ray
CCFLAGS = -Wall -Wextra -std=gnu99
SRC = raytracer.c vecmat.c msg.c
OBJS = raytracer.o
OUT = reference.png custom.png

${EXEC}: ${OBJS}
	${CC} ${CCFLAGS} -o ${EXEC} ${OBJS} -lm

${OBJS}:
	${CC} ${CCFLAGS} -c ${SRC}

run: ${EXEC}
	./${EXEC}

clean:
	rm -f ${EXEC} ${OBJS} ${OUT}

raytracer.o:raytracer.c
