# see https://stackoverflow.com/questions/52034997/how-to-make-makefile-recompile-when-a-header-file-is-changed

BASEDIR := .
IDIR := $(BASEDIR)/include
SRC_DIR := $(BASEDIR)/src
OBJ_DIR := $(BASEDIR)/out
EXE := $(BASEDIR)/main

CXX=g++
CXXFLAGS=-I$(IDIR) -pthread -g

LIBS=-lm
LDFLAGS :=


# Probably don't need to change anything below this

SRC_FILES := $(wildcard $(SRC_DIR)/*.cpp) $(wildcard $(SRC_DIR)/**/*.cpp) 
OBJ_FILES := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRC_FILES))
DEP_FILES := $(patsubst $(SRC_DIR)/%.cpp, %.d, $(SRC_FILES))


quick: $(OBJ_FILES)
	@echo Creating Executable: $(EXE)
	@$(CXX) $(CXXFLAGS) $(LIBS) $(LDFLAGS) -o $(EXE) $^


$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp Makefile
	@mkdir -p $(@D)
	@echo Compiling $<
	@$(CXX) $(CXXFLAGS) -MMD -MP -c -o $@ $<

-include $(DEP_FILES)


all : clean quick  # clean and then generate like normal

clean:
	@rm -Rf $(OBJ_DIR)
	@rm -f $(EXE)
	
