#include "morse.h"
#include <stdlib.h>
#include <string.h>

typedef struct TrieNode {
    char letter;  // 若為有效終點，存字母；否則為 '\0'
    struct TrieNode* dot;
    struct TrieNode* dash;
} TrieNode;

static TrieNode* root = NULL;

// 建立節點
static TrieNode* create_node(void) {
    TrieNode* node = (TrieNode*)malloc(sizeof(TrieNode));
    node->letter = '\0';
    node->dot = NULL;
    node->dash = NULL;
    return node;
}

// 插入字元進 Trie
static void insert(const char* code, char letter) {
    TrieNode* curr = root;
    for (int i = 0; code[i]; ++i) {
        if (code[i] == '.') {
            if (!curr->dot) curr->dot = create_node();
            curr = curr->dot;
        } else if (code[i] == '-') {
            if (!curr->dash) curr->dash = create_node();
            curr = curr->dash;
        }
    }
    curr->letter = letter;
}

// 初始化 Trie Tree：摩斯碼對照表
void morse_trie_init(void) {
    root = create_node();

    insert(".-", 'A');    insert("-...", 'B');  insert("-.-.", 'C'); insert("-..", 'D');
    insert(".", 'E');     insert("..-.", 'F');  insert("--.", 'G');  insert("....", 'H');
    insert("..", 'I');    insert(".---", 'J');  insert("-.-", 'K');  insert(".-..", 'L');
    insert("--", 'M');    insert("-.", 'N');    insert("---", 'O');  insert(".--.", 'P');
    insert("--.-", 'Q');  insert(".-.", 'R');   insert("...", 'S');  insert("-", 'T');
    insert("..-", 'U');   insert("...-", 'V');  insert(".--", 'W');  insert("-..-", 'X');
    insert("-.--", 'Y');  insert("--..", 'Z');

    insert("-----", '0'); insert(".----", '1'); insert("..---", '2'); insert("...--", '3');
    insert("....-", '4'); insert(".....", '5'); insert("-....", '6'); insert("--...", '7');
    insert("---..", '8'); insert("----.", '9');

    insert("/", ' ');  // 字間空白
}

// 查詢摩斯碼對應字元
char morse_lookup(const char* morse_code) {
    TrieNode* curr = root;
    for (int i = 0; morse_code[i]; ++i) {
        if (morse_code[i] == '.') {
            if (!curr->dot) return '?';
            curr = curr->dot;
        } else if (morse_code[i] == '-') {
            if (!curr->dash) return '?';
            curr = curr->dash;
        } else if (morse_code[i] == '/') {
            return ' ';  // 單字分隔符
        } else {
            return '?';  // 不合法的符號
        }
    }
    return curr->letter ? curr->letter : '?';
}
