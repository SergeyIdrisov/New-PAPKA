class Vigenere:
    alphabet = "абвгдеёжзийклмнопрстуфхцчшщъыьэюя"  # TODO

    def __init__(self, keyword):
        self.alphaindex = {ch: index for index, ch in enumerate(self.alphabet)}
        self.key = [self.alphaindex[letter] for letter in keyword.lower()]

    def caesar(self, letter, shift):
        if letter in self.alphaindex:  # строчная буковка
            index = (self.alphaindex[letter] + shift) % len(self.alphabet)
            cipherletter = self.alphabet[index]
        elif letter.lower() in self.alphaindex:  # заглавная буковка
            cipherletter = self.caesar(letter.lower(), shift).upper()
        else:
            cipherletter = letter
        return cipherletter

    def encode(self, line):
        ciphertext = []
        for i, letter in enumerate(line):
            shift = self.key[i % len(self.key)]
            cipherletter = self.caesar(letter, shift)
            ciphertext.append(cipherletter)

        return ''.join(ciphertext)

    def decode(self, line):
        pass  # TODO


keyword = input('keyword=')
cipher = Vigenere(keyword)

line = input()
while line != '.':
    print(cipher.encode(line))
    line = input()
#энигма интересующиеся могут почитать про реализацию сложного полиалфавитного шифра -- компьютерную модель шифра "энигмы". элементы методов шифрования, использовавшихся в машинах этой серии, используются до сих пор.
