# RP2040 SIO除算器の修正（2026-10-12）

状態: **除算器を修正し、回帰確認済み。** 修正の識別は、本記録を含むGitコミットを
参照してください。検証時は変更中の作業ツリーを用い、source hash・差分を保存しました。
本記録の受入対象はエミュレーターの除算器です。アプリの表示やセンサーの観測は
受入条件に含めません。[修正前の調査記録](rp2040_sio_divider_bug_20261012.md)と
[当時の生出力](evidence/sio-divider-review-20261012/README.md)は履歴として保持します。

## 修正した動作

| 対象 | 修正後 |
| --- | --- |
| 計算開始 | 4つのoperand aliasすべてへの書き込みで計算を開始。同値の再書き込みも再開始 |
| 符号選択 | 最後に書いたaliasがsigned／unsignedを選択 |
| READY | operand書き込み直後は0。8sysclk経過で結果を反映し1にする |
| DIRTY | operand／結果の書き込みで1。QUOTIENT読み出しで0、REMAINDER読み出しでは維持 |
| 結果への直接書き込み | 進行中の計算を中断し、指定された結果を保存。READY／DIRTYを1にする |
| core間の独立性 | 各coreがoperand、結果、フラグ、進行中の計算を個別に保持 |
| Threadedの状態引継ぎ | Serialからのpromotionと、複数のworker実行呼び出しをまたいで除算器状態を保存 |

READY=0の間の結果レジスターは仕様上未定義です。このモデルでは前の結果を保持し、
回帰テストは計算完了前の商・余りの値を合格条件にしません。
除数0とsigned overflowの規則は維持し、境界値も確認しています。

## 実装

- [共通のRP2040除算器モデル](../crates/rp2040-emu/src/bus/divider.rs)を
  Serial `Sio`とThreaded `WorkerBus`で使います。RP2350には適用しません。
- `CoreBus::advance_core_cycles`をCPUの命令・例外入口の完了時に呼び、
  quantumの途中でも計算時間を進めます。Serialはschedulerのsystem cycleに
  同期し、WorkerBusはローカルcycleと公開されたsystem cycleを突き合わせます。
  待機中の経過時間を反映し、既に進めたcycleを二重に数えません。
- WorkerBusの生成・終了時にcoreごとの除算器snapshotを引き継ぎます。
  snapshotのlockはMMIOアクセスごとには取得しません。
- 直接 `Sio::read32`／`write32`を呼ぶ利用者は、完了確認までに
  `Sio::advance_divider_cycles(core, cycles)`で時間を進める必要があります。
  CPUの通常実行では自動で進みます。

## 確認

[回帰テスト](../crates/rp2040-emu/tests/sio_divider.rs)は、直接Sio、Serial bus、
WorkerBusそれぞれについて、両core・両符号・両operand順を確認します。
各経路2048通り、計6144通りの計算を、32bit値より広いhost整数の商・余りと照合します。
7cycle時点の未完了、8cycle時点の完了、再開始、同値書き込み、結果書き込みによる中断、
DIRTY、除数0、overflow、context保存・復元を確認します。
CPU命令・例外入口のcycle反映、待機中の完了、時計同期の重複回避、時計のwrapも
別のテストで確認します。

[SDK fixture](../crates/rp2040-emu/tests/fixtures/sio_divider_sdk/README.md)は
固定SDK `a1438dff1d38bd9c65dbd693f0e5db4b9ae91779`の除算assemblyを無変更で
コンパイルしたものです。`y64_x32`／`y64_x48`／`y64_x64`に対応する大きな除数、
正負、overflowなど13入力を、両core・quantum 1／128・Serial／Threadedで
実行します。計104呼び出しで商・余りとdirtyな除算器状態の保存・復元を確認し、
Threadedではworker実行を分割しても状態が継続することを確認します。
原本のfixture BIN、SDK・入力・BINのSHA、buildコマンドとsymbol表を保存しています。

実施したコマンド:

```sh
cargo test --offline -p rp2040-emu
cargo test --offline -p rp2040-emu --features threading,testing
cargo clippy --offline -p rp2040-emu --lib --test sio_divider --features threading,testing -- -D warnings
cargo clippy --offline -p rp2040-emu --all-targets --features threading,testing -- -D warnings -A clippy::identity_op
```

default構成1269件、threading／testing構成1492件が合格しました。
本体と新設integration testは厳格Clippyに合格し、全targetのClippyは既存テストの
`BASE + 0x000`等に対する`identity_op`だけを除外して合格しました。
変更RustファイルのrustfmtとGitの差分検査も確認済みです。
[実行ログとsource識別](evidence/sio-divider-fix-20261012/README.md)を保存しました。

この確認は仕様に基づくエミュレーター回帰確認です。実機との比較は実施していません。
MMIOアクセスの時点は既存CPUの命令単位のスケジュールに従います。
修正検証時点ではtarget registry／accepted backend pin／公開タグを更新していません。
その後の公開版と移行条件は[v0.2.0リリース記録](release_v0.2.0.md)を参照してください。
