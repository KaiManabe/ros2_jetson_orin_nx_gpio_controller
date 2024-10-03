## Jetson Orin NXのGPIOピンヘッダ番号をline番号に変換するやつ
- を、作っておきました
    - GPIOのline番号を参照する処理を高速化するために

### array_[C/Python].txt
- txtの中に配列を宣言するプログラムが入っている
- 宣言された配列のインデックスにpin番号、値にline番号が対応する
- たとえば
    - pin.31のline番号は106
    - pinToArray[31] == 106 となっている

### function_[C/Python].txt
- txtの中に関数を宣言するプログラムが入っている
- 宣言された関数の引数pin番号を渡すと、line番号が返ってくる
- たとえば
    - pin.31のline番号は106
    - pinToArray(31) は 106が返ってくる
