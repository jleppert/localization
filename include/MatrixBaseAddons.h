inline void msgpack_unpack(msgpack::object o) {
    if(o.type != msgpack::type::ARRAY) { throw msgpack::type_error(); }

    msgpack::object * p = o.via.array.ptr;

    std::string type;
    *p >> type;
    if (type != "__eigen__") { throw msgpack::type_error(); }

    size_t rows;
    size_t cols;

    ++p;
    *p >> rows;
    ++p;
    *p >> cols;
    this->resize(rows, cols);

    for (size_t i = 0; i < this->rows(); ++i) {
        for (size_t j = 0; j < this->cols(); ++j) {
            ++p;
            *p >> this->operator()(i, j);
        }
    }
}

template <typename Packer>
inline void msgpack_pack(Packer& pk) const {
    pk.pack_array(3 + this->rows()*this->cols());
    pk.pack(std::string("__eigen__"));
    pk.pack(this->rows());
    pk.pack(this->cols());

    for (size_t i = 0; i < this->rows(); ++i) {
        for (size_t j = 0; j < this->cols(); ++j) {
            pk.pack(this->operator()(i, j));
        }
    }
}

template <typename MSGPACK_OBJECT>
inline void msgpack_object(MSGPACK_OBJECT* o, msgpack::zone* z) const { }
