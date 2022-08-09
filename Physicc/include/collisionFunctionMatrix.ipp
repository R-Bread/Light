// The collisionFunctionMatrix is defined here because it needs to be able to see the full
// definitions of all the functions before it can do its thing
template <typename... CollisionTypes>
class collisionFunctionMatrix {
	public:
		// change this typedef as required
		typedef Contact (*collisionFuncSignature)(Broadphase::PotentialContact);

		collisionFunctionMatrix() {
			// When an object of this struct is instatiated, we will construct the
			// matrix.
			constructMatrix<0, 0, CollisionTypes...>();
		}

		std::array<std::array<collisionFuncSignature, sizeof...(CollisionTypes)>,
				   sizeof...(CollisionTypes)>
			matrix;

		std::array<collisionFuncSignature, sizeof...(CollisionTypes)>& operator[](std::size_t idx) {
				return matrix[idx];
		}

   private:
	  template <typename std::size_t row, std::size_t col, typename Head, typename... Rest>
	  void constructMatrix() {
		  matrix[row][col] = convert<Head, Head>();

		  if constexpr (sizeof...(Rest) > 0) {
						  constructRow<row, col + 1, Head, Rest...>();
						  constructCol<row + 1, col, Head, Rest...>();
			  constructMatrix<row + 1, col + 1, Rest...>();
		  }
	  }

	  template <std::size_t row, std::size_t col, typename Head, typename Next,
				typename... Rest>
	  void constructRow() {
		  matrix[row][col] = convert<Head, Next>();
		  if constexpr (sizeof...(Rest) > 0) {
			  constructRow<row, col + 1, Head, Rest...>();
		  }
	  }

	  template <std::size_t row, std::size_t col, typename Head, typename Next,
				typename... Rest>
	  void constructCol() {
		  matrix[row][col] = convert<Next, Head>();
		  if constexpr (sizeof...(Rest) > 0) {
			  constructCol<row + 1, col, Head, Rest...>();
		  }
	  }
};
