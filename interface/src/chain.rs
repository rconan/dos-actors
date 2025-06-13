/**
Clients chain

Apply the traits [Read], [Update] and [Write] (and in that order)
 to clients which inputs and outputs form an uninterrupted chain

A single client:
```ignore
chain!(
    input_uid: input_data_1,
    ...,
    input_uid: input_data_n;
    client;
    output_uid: output_data_1,
    ...,
    output_uid: output_data_n);
```

Two clients where the UID of the output of `client1` is
also the UID of the input of `client2`:
```ignore
chain!(
    client1_input_uid: client_1_input_data_1,
    ...,
    client1_input_uid: client_1_input_data_n;
    client1;
    io_uid;
    client2;
    io_uid;
    ...;
    clientn:
    clientn_output_uid: client2_output_data_1,
    ...,
    clientn_output_uid: client2_output_data_n);
    );
```

*/
#[macro_export]
macro_rules! chain {
    // read, update
    // UID1: input1, ..., UIDN: inputN; client
    ($($r:ty:$vr:expr),+;$e:expr) => {
        $(<_ as ::interface::Read<$r>>::read($e, interface::Data::<$r>::from($vr));)+
        <_ as ::interface::Update>::update($e);
    };
    // update, write
    // client; UID1: output1, ..., UIDN: outputN
    ($e:expr;$($w:ty:$vw:ident),+) => {
            <_ as ::interface::Update>::update($e);
            $(let $vw = <_ as ::interface::Write<$w>>::write($e).expect(&format!(
                "cannot write to {}",
                ::std::any::type_name::<$w>()
            ));)+
    };
    // read, update, write
    // UID1: input1, ..., UIDN: inputN; client; UID1: output1, ..., UIDN: outputN
    ($($r:ty:$vr:expr),+;$e:expr;$($w:ty:$vw:ident),+) => {
            $(<_ as ::interface::Read<$r>>::read($e, interface::Data::<$r>::from($vr));)+
            <_ as ::interface::Update>::update($e);
            $(let $vw =<_ as ::interface::Write<$w>>::write($e).expect(&format!(
                "cannot write to {}",
                ::std::any::type_name::<$w>()
            ));)+
    };
    // read, update, write, read, update
    // UID1: input1, ..., UIDN: inputN; client1; UID; client2
    ($($r1:ty:$vr1:expr),+;$e1:expr;$w1:ty;$e2:expr) => {
        ::interface::chain!($($r1:$vr1),+;$e1;$w1:data);
        ::interface::chain!($w1:data;$e2);
    };
    // update, write, read, update
    // client1; UID; client2
    ($e1:expr;$w1:ty;$e2:expr) => {
        ::interface::chain!($e1;$w1:data);
        ::interface::chain!($w1:data;$e2);
    };
    // update, write, read, update, write
    // client1; UID; client2; UID1: output1, ..., UIDN: outputN
    ($e1:expr;$w1:ty;$e2:expr;$($w2:ty:$vw2:ident),+) => {
        ::interface::chain!($e1;$w1:data);
        ::interface::chain!($w1:data;$e2;$($w2:$vw2),+);
    };
    // read, update, write, read, update, write
    // UID1: input1, ..., UIDN: inputN; client1; UID; client2; UID1: output1, ..., UIDN: outputN
    ($($r1:ty:$vr1:expr),+;$e1:expr;$w1:ty;$e2:expr;$($w2:ty:$vw2:ident),+) => {
        ::interface::chain!($($r1:$vr1),+;$e1;$w1:data);
        ::interface::chain!($w1:data;$e2;$($w2:$vw2),+);
    };
    // read, update, write, read, update, write, read, update, write
    // UID1: input1, ..., UIDN: inputN; client1; UID; client2; UID; client3; UID1: output1, ..., UIDN: outputN
    ($($r1:ty:$vr1:expr),+;$e1:expr;$w1:ty;$e2:expr;$w2:ty;$e3:expr;$($w3:ty:$vw3:ident),+) => {
        ::interface::chain!($($r1:$vr1),+;$e1;$w1;$e2;$w2:data);
        ::interface::chain!($w2:data;$e3;$($w3:$vw3),+);
    };
    // read, update, write, read, update, write, read, update, write, read, update, write
    // UID1: input1, ..., UIDN: inputN; client1; UID; client2; UID; client3; UID; client4; UID1: output1, ..., UIDN: outputN
    ($($r1:ty:$vr1:expr),+;$e1:expr;$w1:ty;$e2:expr;$w2:ty;$e3:expr;$w3:ty;$e4:expr;$($w4:ty:$vw4:ident),+) => {
        ::interface::chain!($($r1:$vr1),+;$e1;$w1;$e2;$w2;$e3;$w3:data);
        ::interface::chain!($w3:data;$e4;$($w4:$vw4),+);
    };
}
