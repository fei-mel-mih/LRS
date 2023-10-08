namespace lrs
{
    struct PositionStruct
    {
        float x;
        float y;
        
        PositionStruct operator +(PositionStruct const& obj) const
        {
            PositionStruct result;
            result.x = this->x + obj.x;
            result.y = this->y + obj.y;

            return result;
        }
    };

    struct LocationStruct
    {
        float x;
        float y;
        float z;

        LocationStruct operator +(LocationStruct const& obj) const
        {
            LocationStruct result;
            result.x = this->x + obj.x;
            result.y = this->y + obj.y;
            result.z = this->z + obj.z;

            return result;
        }
    };
}